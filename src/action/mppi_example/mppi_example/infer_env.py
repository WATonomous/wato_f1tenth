import os
from pathlib import Path
from functools import partial

import jax
import jax.numpy as jnp
import numpy as np
import yaml
from numba import njit
from PIL import Image
from scipy.ndimage import distance_transform_edt

try:
    from .utils import jax_utils
    from .dynamics_models.dynamics_models_jax import vehicle_dynamics_ks, vehicle_dynamics_st
except ImportError:
    import utils.jax_utils as jax_utils
    from dynamics_models.dynamics_models_jax import vehicle_dynamics_ks, vehicle_dynamics_st

# Lower clip floor for any friction value MPPI uses (embedded per-waypoint or
# scalar fallback) — keeps mu strictly positive so the dynamics never divide by
# a zero/near-zero grip. The UPPER ceiling is the live `friction_max` ROS param
# (config.friction_max); MPPI trims the planned/embedded mu to it at read time,
# so the raceline can carry an aggressive mu and you can raise the cap on the
# fly. Note: the active dynamic_ST model treats mu as a free linear scale, so
# there is no tire-curve limit here — friction_max is a tunable safety ceiling,
# not a hard model bound. Default lives in mppi_node defaults / config.yaml.
FRICTION_MIN = 1e-3
FRICTION_MAX_DEFAULT = 1.5

CUDANUM = 0
# Preallocate a fixed slice of unified memory so the JAX allocator doesn't
# fragment over a long race. The original 0.45 (~3.3 GB of the 7.4 GB Orin
# pool) assumed a race-day container running only PF + SICK + system.
# On this dev box the same container also runs rviz2 plus a VS Code remote
# server / Claude Code session riding along for development, which alone
# left only ~2.6-3.0 GB free before mppi_node even started (measured via
# `free -h`) -- so 0.45 reliably hit NvMapMemAllocInternalTagged ENOMEM
# (error 12). Lowered to fit that measured headroom; raise it back once
# running in a dev-tooling-free container (e.g. actual race day).
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "true"
os.environ["XLA_PYTHON_CLIENT_MEM_FRACTION"] = "0.4"
os.environ["CUDA_VISIBLE_DEVICES"] = str(CUDANUM)

class InferEnv():
    def __init__(self, track, config, DT,
                 jrng=None, dyna_config=None) -> None:
        self.a_shape = 2
        self.track = track
        self.waypoints = track.waypoints
        self.diff = self.waypoints[1:, 1:3] - self.waypoints[:-1, 1:3]
        self.waypoints_distances = np.linalg.norm(self.waypoints[1:, (1, 2)] - self.waypoints[:-1, (1, 2)], axis=1)
        # Per-waypoint friction: pull from track if CSV had column 10, else broadcast config.friction
        # so downstream code can always index `self.waypoint_frictions[i]`.
        # Store the RAW per-waypoint friction; the clip to [FRICTION_MIN,
        # config.friction_max] is applied at read time in get_reference_frictions
        # so the live `friction_max` param takes effect without a re-push.
        track_fric = getattr(track, "frictions", None)
        if track_fric is not None and np.isfinite(track_fric).all():
            self.waypoint_frictions = np.asarray(track_fric, dtype=np.float32)
            self.has_per_waypoint_friction = True
            print(f"InferEnv: per-waypoint friction loaded ({len(self.waypoint_frictions)} pts, "
                  f"raw min={self.waypoint_frictions.min():.3f}, max={self.waypoint_frictions.max():.3f}; "
                  f"clipped to [{FRICTION_MIN}, friction_max] at use)")
        else:
            self.waypoint_frictions = np.full(len(self.waypoints), float(config.friction), dtype=np.float32)
            self.has_per_waypoint_friction = False
        self.reference = None
        self.DT = DT
        self.config = config
        self.jrng = jax_utils.oneLineJaxRNG(0) if jrng is None else jrng
        self.state_frenet = jnp.zeros(6)
        self.norm_params = config.norm_params
        print('MPPI Model:', self.config.state_predictor)
        
        def RK4_fn(x0, u, Ddt, vehicle_dynamics_fn, args):
            # return x0 + vehicle_dynamics_fn(x0, u, *args) * Ddt # Euler integration
            # RK4 integration
            k1 = vehicle_dynamics_fn(x0, u, *args)
            k2 = vehicle_dynamics_fn(x0 + k1 * 0.5 * Ddt, u, *args)
            k3 = vehicle_dynamics_fn(x0 + k2 * 0.5 * Ddt, u, *args)
            k4 = vehicle_dynamics_fn(x0 + k3 * Ddt, u, *args)
            return x0 + (k1 + 2 * k2 + 2 * k3 + k4) / 6 * Ddt
            
        if self.config.state_predictor == 'dynamic_ST':
            @jax.jit
            def update_fn(x, u, friction):
                x1 = x.copy()
                Ddt = 0.1
                def step_fn(i, x0):
                    args = (friction,)
                    return RK4_fn(x0, u, Ddt, vehicle_dynamics_st, args)
                x1 = jax.lax.fori_loop(0, int(self.DT/Ddt), step_fn, x1)
                return (x1, 0, x1-x)
            self.update_fn = update_fn
            
        elif self.config.state_predictor == 'kinematic_ST':
            @jax.jit
            def update_fn(x, u, friction):
                x_k = x.copy()[:5]
                Ddt = 0.1
                def step_fn(i, x0):
                    args = ()
                    return RK4_fn(x0, u, Ddt, vehicle_dynamics_ks, args)
                x_k = jax.lax.fori_loop(0, int(self.DT/Ddt), step_fn, x_k)
                x1 = x.at[:5].set(x_k)
                return (x1, 0, x1-x)
            self.update_fn = update_fn

        self._init_wall_cost()
            
    @partial(jax.jit, static_argnums=(0,))
    def step(self, x, u, rng_key=None, norm_params=None, friction=None):
        if norm_params is None:
            norm_params = self.norm_params
        if friction is None:
            friction = self.config.friction
        return self.update_fn(x, u * norm_params[0, :2]/2, friction)
    
    @partial(jax.jit, static_argnums=(0,))
    def reward_fn_sey(self, s, reference, reward_weights=None):
        """
        reward function for the state s with respect to the reference trajectory
        """
        if reward_weights is None:
            reward_weights = jnp.array([1.0, 0.0, 0.0])
        invalid_penalty = (~jnp.isfinite(s).all(axis=1)).astype(jnp.float32) * 1e3
        s_safe = jnp.nan_to_num(s, nan=1e3, posinf=1e3, neginf=-1e3)
        sey_reward = -jnp.linalg.norm(reference[1:, 4:6] - s_safe[:, :2], ord=1, axis=1)
        vel_reward = -jnp.abs(reference[1:, 2] - s_safe[:, 3])
        yaw_reward = -jnp.abs(jnp.sin(reference[1:, 3]) - jnp.sin(s_safe[:, 4])) - \
            jnp.abs(jnp.cos(reference[1:, 3]) - jnp.cos(s_safe[:, 4]))
            
        return reward_weights[0] * sey_reward + reward_weights[1] * vel_reward + reward_weights[2] * yaw_reward - invalid_penalty
    
    def update_waypoints(self, waypoints):
        self.waypoints = waypoints
        self.diff = self.waypoints[1:, 1:3] - self.waypoints[:-1, 1:3]
        self.waypoints_distances = np.linalg.norm(self.waypoints[1:, (1, 2)] - self.waypoints[:-1, (1, 2)], axis=1)
        # Re-read friction column if present (10th column). Stored raw; the
        # [FRICTION_MIN, friction_max] clip is applied at read time so a live
        # friction_max change re-trims a hot-swapped traj without a re-push.
        if waypoints.shape[1] >= 10:
            fric = waypoints[:, 9]
            if np.isfinite(fric).all():
                self.waypoint_frictions = np.asarray(fric, dtype=np.float32)
                self.has_per_waypoint_friction = True
                return
        # Fall back to scalar broadcast
        self.waypoint_frictions = np.full(len(waypoints), float(self.config.friction), dtype=np.float32)
        self.has_per_waypoint_friction = False

    def get_reference_frictions(self, state, n_steps):
        """Return per-step friction along the upcoming reference horizon.

        Uses nearest-waypoint-index lookup from the same starting point as
        `get_refernece_traj`. Falls back to scalar `config.friction` array if no
        per-waypoint friction is loaded. Always returns a `(n_steps,)` jnp array.
        """
        # Live grip ceiling: trim the planned/embedded mu to friction_max so
        # MPPI never believes in more grip than allowed. Read each call so the
        # ROS param takes effect instantly.
        fric_max = float(getattr(self.config, 'friction_max', FRICTION_MAX_DEFAULT))
        if not self.has_per_waypoint_friction:
            mu = float(np.clip(self.config.friction, FRICTION_MIN, fric_max))
            return jnp.full((int(n_steps),), mu, dtype=jnp.float32)
        _, _, _, _, ind = nearest_point(np.array([state[0], state[1]]),
                                        self.waypoints[:, (1, 2)].copy(), self.diff)
        N = len(self.waypoint_frictions)
        # Walk indices forward — one waypoint per horizon step (matches the
        # spatial coarseness of the reference; finer-grained interpolation is
        # not worth the complexity given mu changes slowly along the track).
        idxs = (int(ind) + np.arange(int(n_steps))) % N
        mu = np.clip(self.waypoint_frictions[idxs], FRICTION_MIN, fric_max)
        return jnp.asarray(mu, dtype=jnp.float32)
    
    @partial(jax.jit, static_argnums=(0,))
    def reward_fn_xy(self, state, reference, reward_weights=None, cost_params=None, opponent_traj=None):
        """
        reward function for the state s with respect to the reference trajectory
        """
        if reward_weights is None:
            reward_weights = jnp.array([1.0, 0.0, 0.0])
        if cost_params is None:
            cost_params = jnp.zeros((20,), dtype=jnp.float32)
        if opponent_traj is None:
            opponent_traj = jnp.zeros((state.shape[0], 2), dtype=jnp.float32)
        invalid_penalty = (~jnp.isfinite(state).all(axis=1)).astype(jnp.float32) * 1e3
        state_safe = jnp.nan_to_num(state, nan=1e3, posinf=1e3, neginf=-1e3)
        xy_reward = -jnp.linalg.norm(reference[1:, :2] - state_safe[:, :2], ord=1, axis=1)
        vel_reward = -jnp.abs(reference[1:, 2] - state_safe[:, 3])
        yaw_reward = -jnp.abs(jnp.sin(reference[1:, 3]) - jnp.sin(state_safe[:, 4])) - \
            jnp.abs(jnp.cos(reference[1:, 3]) - jnp.cos(state_safe[:, 4]))
        
        reward = (
            reward_weights[0] * xy_reward 
            + reward_weights[1] * vel_reward 
            + reward_weights[2] * yaw_reward
            - invalid_penalty
        )

        wall_weight = cost_params[0]
        wall_margin = cost_params[1]
        wall_power = cost_params[2]
        slip_weight = cost_params[3]
        beta_safe = cost_params[4]
        latacc_weight = cost_params[5]
        latacc_safe = cost_params[6]
        steer_sat_weight = cost_params[7]
        steer_soft = cost_params[8]
        opponent_weight = cost_params[9]
        opponent_radius = cost_params[10]
        opponent_power = cost_params[11]
        opponent_discount = cost_params[12]
        opponent_mode = cost_params[13]
        opponent_follow_weight = cost_params[14]
        opponent_follow_distance = cost_params[15]
        opponent_same_lane_width = cost_params[16]
        opponent_pass_weight = cost_params[17]
        opponent_pass_lateral_offset = cost_params[18]
        opponent_pass_longitudinal_window = cost_params[19]

        if self.wall_sdf is not None:
            wall_dist = self.sample_wall_distance(state_safe[:, :2])
            wall_penalty = jnp.maximum(0.0, wall_margin - wall_dist) ** wall_power
            reward -= wall_weight * wall_penalty

        beta = jnp.abs(state_safe[:, 6])
        slip_penalty = self.safe_hinge_square(beta, beta_safe)
        reward -= slip_weight * slip_penalty
        
        latacc = jnp.abs(state_safe[:, 3] * state_safe[:, 5])
        latacc_penalty = self.safe_hinge_square(latacc, latacc_safe)
        reward -= latacc_weight * latacc_penalty

        steer_abs = jnp.abs(state_safe[:, 2])
        steer_penalty = self.safe_hinge_square(steer_abs, steer_soft)
        reward -= steer_sat_weight * steer_penalty

        opponent_xy = opponent_traj[:state_safe.shape[0], :2]
        opponent_dist = jnp.linalg.norm(state_safe[:, :2] - opponent_xy, axis=1)
        opponent_penalty = jnp.maximum(0.0, opponent_radius - opponent_dist) ** opponent_power
        opponent_discount_steps = opponent_discount ** jnp.arange(state_safe.shape[0], dtype=jnp.float32)
        reward -= opponent_weight * opponent_discount_steps * opponent_penalty

        ref_yaw = reference[1:state_safe.shape[0] + 1, 3]
        tangent = jnp.stack([jnp.cos(ref_yaw), jnp.sin(ref_yaw)], axis=1)
        normal = jnp.stack([-jnp.sin(ref_yaw), jnp.cos(ref_yaw)], axis=1)
        rel_xy = state_safe[:, :2] - opponent_xy
        long_rel = jnp.sum(rel_xy * tangent, axis=1)
        lat_rel = jnp.sum(rel_xy * normal, axis=1)

        same_lane_scale = jnp.maximum(opponent_same_lane_width, 1e-3)
        same_lane_factor = jnp.maximum(0.0, 1.0 - jnp.abs(lat_rel) / same_lane_scale)
        behind_mask = (long_rel < 0.0).astype(jnp.float32)
        follow_violation = jnp.maximum(0.0, opponent_follow_distance + long_rel)
        follow_active = (jnp.abs(opponent_mode - 0.0) < 0.5).astype(jnp.float32)
        follow_penalty = follow_active * behind_mask * same_lane_factor * jnp.square(follow_violation)
        reward -= opponent_follow_weight * opponent_discount_steps * follow_penalty

        pass_left_active = (jnp.abs(opponent_mode - 2.0) < 0.5).astype(jnp.float32)
        pass_right_active = (jnp.abs(opponent_mode - 3.0) < 0.5).astype(jnp.float32)
        pass_side = pass_left_active - pass_right_active
        pass_active = pass_left_active + pass_right_active
        pass_window = jnp.maximum(opponent_pass_longitudinal_window, 1e-3)
        pass_window_factor = jnp.maximum(0.0, 1.0 - jnp.abs(long_rel) / pass_window)
        pass_violation = jnp.maximum(0.0, opponent_pass_lateral_offset - pass_side * lat_rel)
        pass_penalty = pass_active * pass_window_factor * jnp.square(pass_violation)
        reward -= opponent_pass_weight * opponent_discount_steps * pass_penalty
            
        return reward

    @partial(jax.jit, static_argnums=(0,))
    def safe_hinge_square(self, value, threshold, cap=1e3):
        violation = jnp.maximum(0.0, value - threshold)
        violation = jnp.minimum(violation, cap)
        return jnp.square(violation)
    
    
    def calc_ref_trajectory_kinematic(self, state, cx, cy, cyaw, sp):
        """
        calc referent trajectory ref_traj in T steps: [x, y, v, yaw]
        using the current velocity, calc the T points along the reference path
        :param cx: Course X-Position
        :param cy: Course y-Position
        :param cyaw: Course Heading
        :param sp: speed profile
        :dl: distance step
        :pind: Setpoint Index
        :return: reference trajectory ref_traj, reference steering angle
        """

        n_state = 4
        n_steps = 10
        # Create placeholder Arrays for the reference trajectory for T steps
        ref_traj = np.zeros((n_state, n_steps + 1))
        ncourse = len(cx)

        # Find nearest index/setpoint from where the trajectories are calculated
        _, _, _, ind = nearest_point(np.array([state.x, state.y]), np.array([cx, cy]).T)

        # Load the initial parameters from the setpoint into the trajectory
        ref_traj[0, 0] = cx[ind]
        ref_traj[1, 0] = cy[ind]
        ref_traj[2, 0] = sp[ind]
        ref_traj[3, 0] = cyaw[ind]

        # based on current velocity, distance traveled on the ref line between time steps
        travel = abs(state.v) * self.config.DTK
        dind = travel / self.config.dlk
        ind_list = int(ind) + np.insert(
            np.cumsum(np.repeat(dind, self.config.TK)), 0, 0
        ).astype(int)
        ind_list[ind_list >= ncourse] -= ncourse
        ref_traj[0, :] = cx[ind_list]
        ref_traj[1, :] = cy[ind_list]
        ref_traj[2, :] = sp[ind_list]
        cyaw[cyaw - state.yaw > 4.5] = np.abs(
            cyaw[cyaw - state.yaw > 4.5] - (2 * np.pi)
        )
        cyaw[cyaw - state.yaw < -4.5] = np.abs(
            cyaw[cyaw - state.yaw < -4.5] + (2 * np.pi)
        )
        ref_traj[3, :] = cyaw[ind_list]

        return ref_traj
    
    @partial(jax.jit, static_argnums=(0,3))
    def get_refernece_traj_jax(self, state, target_speed, n_steps=10):
        _, dist, _, _, ind = nearest_point_jax(jnp.array([state[0], state[1]]), 
                                           self.waypoints[:, (1, 2)], jnp.array(self.diff))
        
        speed = target_speed
        speeds = jnp.ones(n_steps) * speed
        
        reference = get_reference_trajectory_jax(speeds, dist, ind, 
                                            self.waypoints.copy(), int(n_steps),
                                            self.waypoints_distances.copy(), DT=self.DT)
        orientation = state[4]
        reference = reference.at[:, 3].set(
            jnp.where(reference[:, 3] - orientation > 5, 
                  reference[:, 3] - 2 * jnp.pi, 
                  reference[:, 3])
        )
        reference = reference.at[:, 3].set(
            jnp.where(reference[:, 3] - orientation < -5, 
                  reference[:, 3] + 2 * jnp.pi, 
                  reference[:, 3])
        )
        
        return reference, ind
    
    def _profiled_reference_speeds(self, reference):
        speeds = np.asarray(reference[:, 2], dtype=float)
        scale = float(getattr(self.config, 'speed_profile_scale', 1.0))
        min_speed = float(getattr(self.config, 'speed_profile_min_speed', 0.0))
        max_speed = float(getattr(self.config, 'speed_profile_max_speed', 20.0))
        max_speed = max(min_speed, max_speed)

        speeds = np.clip(speeds * scale, min_speed, max_speed)

        lookahead_steps = max(0, int(getattr(self.config, 'speed_profile_lookahead_steps', 0)))
        if lookahead_steps > 0 and speeds.size > 1:
            padded = np.pad(speeds, (0, lookahead_steps), mode='edge')
            speeds = np.asarray([
                np.min(padded[i:i + lookahead_steps + 1])
                for i in range(speeds.size)
            ])
        return speeds

    def get_refernece_traj(self, state, target_speed=None, n_steps=10, vind=5, speed_factor=1.0):
        _, dist, _, _, ind = nearest_point(np.array([state[0], state[1]]), 
                                           self.waypoints[:, (1, 2)].copy(), self.diff)
        
        if target_speed is None:
            # speed = self.waypoints[ind, vind] * speed_factor
            # speed = np.minimum(self.waypoints[ind, vind] * speed_factor, 20.)
            speed = state[3]
        else:
            speed = target_speed

        if getattr(self.config, 'use_waypoint_speed_profile', False):
            seed_reference = np.zeros((1, 7))
            seed_reference[0, 2] = self.waypoints[ind, vind]
            speed = self._profiled_reference_speeds(seed_reference)[0]
        
        speeds = np.ones(n_steps) * speed
        
        reference = get_reference_trajectory(speeds, dist, ind, 
                                            self.waypoints.copy(), int(n_steps),
                                            self.waypoints_distances.copy(), DT=self.DT)
        if getattr(self.config, 'use_waypoint_speed_profile', False):
            # The raceline CSV already carries vx_mps in column 5. Use a light
            # fixed-point pass so tight-corner speed targets also shorten the
            # spatial spacing of the future reference, not just its velocity column.
            for _ in range(max(1, int(getattr(self.config, 'speed_profile_iterations', 1)))):
                speed_targets = self._profiled_reference_speeds(reference.T)
                speeds = speed_targets[1:n_steps + 1]
                reference = get_reference_trajectory(speeds, dist, ind,
                                                    self.waypoints.copy(), int(n_steps),
                                                    self.waypoints_distances.copy(), DT=self.DT)
            reference[2, :] = self._profiled_reference_speeds(reference.T)

        orientation = state[4]
        reference[3, :][reference[3, :] - orientation > 5] = np.abs(
            reference[3, :][reference[3, :] - orientation > 5] - (2 * np.pi))
        reference[3, :][reference[3, :] - orientation < -5] = np.abs(
            reference[3, :][reference[3, :] - orientation < -5] + (2 * np.pi))
        
        # reference[2] = np.where(reference[2] - speed > 5.0, speed + 5.0, reference[2])
        self.reference = reference.T
        return reference.T, ind
    
    def _init_wall_cost(self):
        needs_wall_sdf = (
            bool(getattr(self.config, 'wall_cost_enabled', False))
            or bool(getattr(self.config, 'opponent_auto_wall_check_enabled', False))
        )
        signature = (
            needs_wall_sdf,
            str(getattr(self.config, 'wall_cost_map_yaml', '')),
        )
        if getattr(self, 'wall_cost_signature', None) == signature:
            return

        self.wall_sdf = None
        self.wall_origin = None
        self.wall_resolution = None
        self.wall_cost_signature = signature

        if not needs_wall_sdf:
            return
        map_yaml = getattr(self.config, 'wall_cost_map_yaml', '')
        if not map_yaml:
            return
        
        sdf, origin_xy, resolution = load_wall_distance_field(map_yaml)
        self.wall_sdf = jnp.asarray(sdf, dtype=jnp.float32)
        self.wall_origin = jnp.asarray(origin_xy, dtype=jnp.float32)
        self.wall_resolution = float(resolution)

    @partial(jax.jit, static_argnums=(0,))
    def sample_wall_distance(self, xy):
        if self.wall_sdf is None:
            return jnp.ones((xy.shape[0],), dtype=jnp.float32) * 100.0

        col = jnp.floor((xy[:, 0] - self.wall_origin[0]) / self.wall_resolution).astype(jnp.int32)
        row = jnp.floor((xy[:, 1] - self.wall_origin[1]) / self.wall_resolution).astype(jnp.int32)

        h, w = self.wall_sdf.shape
        valid = jnp.logical_and(row >= 0, row < h)
        valid = jnp.logical_and(valid, col >= 0)
        valid = jnp.logical_and(valid, col < w)
        row = jnp.clip(row, 0, h-1)
        col = jnp.clip(col, 0, w-1)

        return jnp.where(valid, self.wall_sdf[row, col], 0.0)

    def reward_debug_terms(self, state, reference, reward_weights=None, cost_params=None, opponent_traj=None):
        if reward_weights is None:
            reward_weights = np.asarray([
                getattr(self.config, 'xy_reward_weight', 1.0),
                getattr(self.config, 'velocity_reward_weight', 0.0),
                getattr(self.config, 'yaw_reward_weight', 0.0),
            ], dtype=np.float32)
        else:
            reward_weights = np.asarray(reward_weights, dtype=np.float32)

        if cost_params is None:
            cost_params = np.asarray([
                getattr(self.config, 'wall_cost_weight', 0.0) if getattr(self.config, 'wall_cost_enabled', False) else 0.0,
                getattr(self.config, 'wall_cost_margin', 0.0),
                getattr(self.config, 'wall_cost_power', 2.0),
                getattr(self.config, 'slip_cost_weight', 0.0) if getattr(self.config, 'slip_cost_enabled', False) else 0.0,
                getattr(self.config, 'slip_cost_beta_safe', 0.0),
                getattr(self.config, 'latacc_cost_weight', 0.0) if getattr(self.config, 'latacc_cost_enabled', False) else 0.0,
                getattr(self.config, 'latacc_cost_safe', 0.0),
                getattr(self.config, 'steer_sat_cost_weight', 0.0) if getattr(self.config, 'steer_sat_cost_enabled', False) else 0.0,
                getattr(self.config, 'steer_sat_soft_ratio', 0.0) * getattr(self.config, 'max_steering_angle', 0.0),
                getattr(self.config, 'opponent_cost_weight', 0.0) if getattr(self.config, 'opponent_cost_enabled', False) else 0.0,
                getattr(self.config, 'opponent_cost_radius', 0.8),
                getattr(self.config, 'opponent_cost_power', 2.0),
                getattr(self.config, 'opponent_cost_discount', 1.0),
                getattr(self.config, 'opponent_behavior_mode_id', 0.0),
                getattr(self.config, 'opponent_follow_weight', 0.0),
                getattr(self.config, 'opponent_follow_distance', 1.2),
                getattr(self.config, 'opponent_same_lane_width', 0.7),
                getattr(self.config, 'opponent_pass_weight', 0.0),
                getattr(self.config, 'opponent_pass_lateral_offset', 0.55),
                getattr(self.config, 'opponent_pass_longitudinal_window', 1.5),
            ], dtype=np.float32)
        else:
            cost_params = np.asarray(cost_params, dtype=np.float32)

        state = np.asarray(state, dtype=np.float32)
        reference = np.asarray(reference, dtype=np.float32)

        if state.ndim != 2 or state.shape[0] == 0:
            return {
                'reward_total_sum': 0.0,
                'reward_total_mean': 0.0,
                'reward_xy_sum': 0.0,
                'reward_velocity_sum': 0.0,
                'reward_yaw_sum': 0.0,
                'cost_total_sum': 0.0,
                'cost_total_mean': 0.0,
                'cost_invalid_sum': 0.0,
                'cost_wall_sum': 0.0,
                'cost_slip_sum': 0.0,
                'cost_latacc_sum': 0.0,
                'cost_steer_sat_sum': 0.0,
                'cost_opponent_sum': 0.0,
                'cost_opponent_follow_sum': 0.0,
                'cost_opponent_pass_sum': 0.0,
                'min_wall_dist': 100.0,
                'min_opponent_dist': 100.0,
                'min_opponent_longitudinal_rel': 0.0,
                'max_abs_opponent_lateral_rel': 0.0,
                'max_beta': 0.0,
                'max_latacc': 0.0,
                'max_abs_steer': 0.0,
                'invalid_steps': 0.0,
            }

        ref = reference[1:]
        n = min(state.shape[0], ref.shape[0])
        state = state[:n]
        ref = ref[:n]

        invalid_penalty = (~np.isfinite(state).all(axis=1)).astype(np.float32) * 1e3
        state_safe = np.nan_to_num(state, nan=1e3, posinf=1e3, neginf=-1e3)

        xy_reward = -np.linalg.norm(ref[:, :2] - state_safe[:, :2], ord=1, axis=1)
        vel_reward = -np.abs(ref[:, 2] - state_safe[:, 3])
        yaw_reward = -np.abs(np.sin(ref[:, 3]) - np.sin(state_safe[:, 4])) - \
            np.abs(np.cos(ref[:, 3]) - np.cos(state_safe[:, 4]))

        weighted_xy = reward_weights[0] * xy_reward
        weighted_vel = reward_weights[1] * vel_reward
        weighted_yaw = reward_weights[2] * yaw_reward

        wall_weight = float(cost_params[0])
        wall_margin = float(cost_params[1])
        wall_power = float(cost_params[2])
        slip_weight = float(cost_params[3])
        beta_safe = float(cost_params[4])
        latacc_weight = float(cost_params[5])
        latacc_safe = float(cost_params[6])
        steer_sat_weight = float(cost_params[7])
        steer_soft = float(cost_params[8])
        opponent_weight = float(cost_params[9])
        opponent_radius = float(cost_params[10])
        opponent_power = float(cost_params[11])
        opponent_discount = float(cost_params[12])
        opponent_mode = float(cost_params[13])
        opponent_follow_weight = float(cost_params[14])
        opponent_follow_distance = float(cost_params[15])
        opponent_same_lane_width = float(cost_params[16])
        opponent_pass_weight = float(cost_params[17])
        opponent_pass_lateral_offset = float(cost_params[18])
        opponent_pass_longitudinal_window = float(cost_params[19])

        if self.wall_sdf is not None:
            wall_dist = np.asarray(self.sample_wall_distance(jnp.asarray(state_safe[:, :2])))
        else:
            wall_dist = np.ones((n,), dtype=np.float32) * 100.0
        wall_penalty = np.maximum(0.0, wall_margin - wall_dist) ** wall_power
        weighted_wall = wall_weight * wall_penalty

        beta = np.abs(state_safe[:, 6])
        slip_penalty = np.square(np.minimum(np.maximum(0.0, beta - beta_safe), 1e3))
        weighted_slip = slip_weight * slip_penalty

        latacc = np.abs(state_safe[:, 3] * state_safe[:, 5])
        latacc_penalty = np.square(np.minimum(np.maximum(0.0, latacc - latacc_safe), 1e3))
        weighted_latacc = latacc_weight * latacc_penalty

        steer_abs = np.abs(state_safe[:, 2])
        steer_penalty = np.square(np.minimum(np.maximum(0.0, steer_abs - steer_soft), 1e3))
        weighted_steer = steer_sat_weight * steer_penalty

        if opponent_traj is None:
            opponent_xy = np.zeros((n, 2), dtype=np.float32)
        else:
            opponent_xy = np.asarray(opponent_traj, dtype=np.float32)[:n, :2]
        if opponent_xy.shape[0] < n:
            pad = np.repeat(opponent_xy[-1:, :], n - opponent_xy.shape[0], axis=0) if opponent_xy.shape[0] else np.zeros((n, 2), dtype=np.float32)
            opponent_xy = np.vstack([opponent_xy, pad])
        opponent_dist = np.linalg.norm(state_safe[:, :2] - opponent_xy, axis=1)
        opponent_penalty = np.maximum(0.0, opponent_radius - opponent_dist) ** opponent_power
        opponent_discount_steps = opponent_discount ** np.arange(n, dtype=np.float32)
        weighted_opponent = opponent_weight * opponent_discount_steps * opponent_penalty

        ref_yaw = ref[:, 3]
        tangent = np.stack([np.cos(ref_yaw), np.sin(ref_yaw)], axis=1)
        normal = np.stack([-np.sin(ref_yaw), np.cos(ref_yaw)], axis=1)
        rel_xy = state_safe[:, :2] - opponent_xy
        long_rel = np.sum(rel_xy * tangent, axis=1)
        lat_rel = np.sum(rel_xy * normal, axis=1)

        same_lane_scale = max(opponent_same_lane_width, 1e-3)
        same_lane_factor = np.maximum(0.0, 1.0 - np.abs(lat_rel) / same_lane_scale)
        behind_mask = (long_rel < 0.0).astype(np.float32)
        follow_violation = np.maximum(0.0, opponent_follow_distance + long_rel)
        follow_active = 1.0 if abs(opponent_mode - 0.0) < 0.5 else 0.0
        weighted_opponent_follow = (
            opponent_follow_weight
            * opponent_discount_steps
            * follow_active
            * behind_mask
            * same_lane_factor
            * np.square(follow_violation)
        )

        pass_left_active = 1.0 if abs(opponent_mode - 2.0) < 0.5 else 0.0
        pass_right_active = 1.0 if abs(opponent_mode - 3.0) < 0.5 else 0.0
        pass_side = pass_left_active - pass_right_active
        pass_active = pass_left_active + pass_right_active
        pass_window = max(opponent_pass_longitudinal_window, 1e-3)
        pass_window_factor = np.maximum(0.0, 1.0 - np.abs(long_rel) / pass_window)
        pass_violation = np.maximum(0.0, opponent_pass_lateral_offset - pass_side * lat_rel)
        weighted_opponent_pass = (
            opponent_pass_weight
            * opponent_discount_steps
            * pass_active
            * pass_window_factor
            * np.square(pass_violation)
        )

        total = (
            weighted_xy
            + weighted_vel
            + weighted_yaw
            - invalid_penalty
            - weighted_wall
            - weighted_slip
            - weighted_latacc
            - weighted_steer
            - weighted_opponent
            - weighted_opponent_follow
            - weighted_opponent_pass
        )
        total_cost = (
            invalid_penalty
            + weighted_wall
            + weighted_slip
            + weighted_latacc
            + weighted_steer
            + weighted_opponent
            + weighted_opponent_follow
            + weighted_opponent_pass
        )

        return {
            'reward_total_sum': float(np.sum(total)),
            'reward_total_mean': float(np.mean(total)),
            'reward_xy_sum': float(np.sum(weighted_xy)),
            'reward_velocity_sum': float(np.sum(weighted_vel)),
            'reward_yaw_sum': float(np.sum(weighted_yaw)),
            'cost_total_sum': float(np.sum(total_cost)),
            'cost_total_mean': float(np.mean(total_cost)),
            'cost_invalid_sum': float(np.sum(invalid_penalty)),
            'cost_wall_sum': float(np.sum(weighted_wall)),
            'cost_slip_sum': float(np.sum(weighted_slip)),
            'cost_latacc_sum': float(np.sum(weighted_latacc)),
            'cost_steer_sat_sum': float(np.sum(weighted_steer)),
            'cost_opponent_sum': float(np.sum(weighted_opponent)),
            'cost_opponent_follow_sum': float(np.sum(weighted_opponent_follow)),
            'cost_opponent_pass_sum': float(np.sum(weighted_opponent_pass)),
            'min_wall_dist': float(np.min(wall_dist)),
            'min_opponent_dist': float(np.min(opponent_dist)),
            'min_opponent_longitudinal_rel': float(np.min(long_rel)),
            'max_abs_opponent_lateral_rel': float(np.max(np.abs(lat_rel))),
            'max_beta': float(np.max(beta)),
            'max_latacc': float(np.max(latacc)),
            'max_abs_steer': float(np.max(steer_abs)),
            'invalid_steps': float(np.count_nonzero(invalid_penalty > 0.0)),
        }

def load_wall_distance_field(map_yaml):
    map_yaml = Path(map_yaml).expanduser().resolve()
    with map_yaml.open('r', encoding='utf-8') as stream:
        map_cfg = yaml.safe_load(stream)

    image_path = Path(map_cfg['image'])
    if not image_path.is_absolute():
        image_path = (map_yaml.parent / image_path).resolve()

    image = np.array(Image.open(image_path).convert('L'), dtype=np.uint8)
    image = np.flipud(image)

    resolution = float(map_cfg['resolution'])
    origin_xy = np.asarray(map_cfg['origin'][:2], dtype=np.float32)
    negate = int(map_cfg.get('negate', 0))
    occupied_thresh = float(map_cfg.get('occupied_thresh', 0.65))

    if negate:
        occ_prob = image.astype(np.float32) / 255.0
    else:
        occ_prob = (255.0 - image.astype(np.float32)) / 255.0

    occupied = occ_prob >= occupied_thresh
    wall_sdf = distance_transform_edt(~occupied).astype(np.float32) * resolution
    return wall_sdf, origin_xy, resolution


@jax.jit
def get_reference_trajectory_jax(predicted_speeds, dist_from_segment_start, idx, 
                             waypoints, n_steps, waypoints_distances, DT):
    total_length = jnp.sum(waypoints_distances)
    s_relative = jnp.concatenate([
        jnp.array([dist_from_segment_start]),
        predicted_speeds * DT
    ]).cumsum()
    s_relative = s_relative % total_length  
    rolled_distances = jnp.roll(waypoints_distances, -idx)
    wp_dist_cum = jnp.concatenate([jnp.array([0.0]), jnp.cumsum(rolled_distances)])
    index_relative = jnp.searchsorted(wp_dist_cum, s_relative, side='right') - 1
    index_relative = jnp.clip(index_relative, 0, len(rolled_distances) - 1)
    index_absolute = (idx + index_relative) % (waypoints.shape[0] - 1)
    next_index = (index_absolute + 1) % (waypoints.shape[0] - 1)
    seg_start = wp_dist_cum[index_relative]
    seg_len = rolled_distances[index_relative]
    t = (s_relative - seg_start) / seg_len
    p0 = waypoints[index_absolute][:, 1:3]
    p1 = waypoints[next_index][:, 1:3]
    interpolated_positions = p0 + (p1 - p0) * t[:, jnp.newaxis]
    s0 = waypoints[index_absolute][:, 0]
    s1 = waypoints[next_index][:, 0]
    interpolated_s = (s0 + (s1 - s0) * t) % waypoints[-1, 0]  
    yaw0 = waypoints[index_absolute][:, 3]
    yaw1 = waypoints[next_index][:, 3]
    interpolated_yaw = yaw0 + (yaw1 - yaw0) * t
    interpolated_yaw = (interpolated_yaw + jnp.pi) % (2 * jnp.pi) - jnp.pi
    v0 = waypoints[index_absolute][:, 5]
    v1 = waypoints[next_index][:, 5]
    interpolated_speed = v0 + (v1 - v0) * t
    reference = jnp.stack([
        interpolated_positions[:, 0],
        interpolated_positions[:, 1],
        interpolated_speed,
        interpolated_yaw,
        interpolated_s,
        jnp.zeros_like(interpolated_speed),
        jnp.zeros_like(interpolated_speed)
    ], axis=1)
    return reference

@jax.jit
def nearest_point_jax(point, trajectory, diffs):
    # diffs = trajectory[1:] - trajectory[:-1]                    
    l2s = jnp.sum(diffs**2, axis=1) + 1e-8                    
    dots = jnp.sum((point - trajectory[:-1]) * diffs, axis=1) 
    t = jnp.clip(dots / l2s, 0., 1.)   
    projections = trajectory[:-1] + diffs * t[:, None]
    dists = jnp.linalg.norm(point - projections, axis=1)      
    min_dist_segment = jnp.argmin(dists)                
    dist_from_segment_start = jnp.linalg.norm(diffs[min_dist_segment] * t[min_dist_segment])          
    return projections[min_dist_segment],dist_from_segment_start, dists[min_dist_segment], t[min_dist_segment], min_dist_segment
    



@njit(cache=False)
def nearest_point(point, trajectory, diffs):
    """
    Return the nearest point along the given piecewise linear trajectory.
    Args:
        point (numpy.ndarray, (2, )): (x, y) of current pose
        trajectory (numpy.ndarray, (N, 2)): array of (x, y) trajectory waypoints
            NOTE: points in trajectory must be unique. If they are not unique, a divide by 0 error will destroy the world
    Returns:
        nearest_point (numpy.ndarray, (2, )): nearest point on the trajectory to the point
        nearest_dist (float): distance to the nearest point
        t (float): nearest point's location as a segment between 0 and 1 on the vector formed by the closest two points on the trajectory. (p_i---*-------p_i+1)
        i (int): index of nearest point in the array of trajectory waypoints
    """
    # diffs = trajectory[1:, :] - trajectory[:-1, :]
    # diffs = self.diff
    l2s = diffs[:, 0] ** 2 + diffs[:, 1] ** 2
    dots = np.empty((trajectory.shape[0] - 1,))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / (l2s + 1e-8)
    t[t < 0.0] = 0.0
    t[t > 1.0] = 1.0
    projections = trajectory[:-1, :] + (t * diffs.T).T
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp * temp))
    min_dist_segment = np.argmin(dists)
    dist_from_segment_start = np.linalg.norm(diffs[min_dist_segment] * t[min_dist_segment])
    return projections[min_dist_segment], dist_from_segment_start, dists[min_dist_segment], t[
        min_dist_segment], min_dist_segment


# @njit(cache=True)
def get_reference_trajectory(predicted_speeds, dist_from_segment_start, idx, 
                             waypoints, n_steps, waypoints_distances, DT):
    s_relative = np.zeros((n_steps + 1,))
    s_relative[0] = dist_from_segment_start
    s_relative[1:] = predicted_speeds * DT
    s_relative = np.cumsum(s_relative)

    waypoints_distances_relative = np.cumsum(np.roll(waypoints_distances, -idx))

    index_relative = np.int_(np.ones((n_steps + 1,)))
    for i in range(n_steps + 1):
        index_relative[i] = (waypoints_distances_relative <= s_relative[i]).sum()
    index_absolute = np.mod(idx + index_relative, waypoints.shape[0] - 1)

    segment_part = s_relative - (
            waypoints_distances_relative[index_relative] - waypoints_distances[index_absolute])

    t = (segment_part / waypoints_distances[index_absolute])
    # print(np.all(np.logical_and((t < 1.0), (t > 0.0))))

    position_diffs = (waypoints[np.mod(index_absolute + 1, waypoints.shape[0] - 1)][:, (1, 2)] -
                        waypoints[index_absolute][:, (1, 2)])
    position_diff_s = (waypoints[np.mod(index_absolute + 1, waypoints.shape[0] - 1)][:, 0] -
                        waypoints[index_absolute][:, 0])
    orientation_diffs = (waypoints[np.mod(index_absolute + 1, waypoints.shape[0] - 1)][:, 3] -
                            waypoints[index_absolute][:, 3])
    speed_diffs = (waypoints[np.mod(index_absolute + 1, waypoints.shape[0] - 1)][:, 5] -
                    waypoints[index_absolute][:, 5])

    interpolated_positions = waypoints[index_absolute][:, (1, 2)] + (t * position_diffs.T).T
    interpolated_s = waypoints[index_absolute][:, 0] + (t * position_diff_s)
    interpolated_s[np.where(interpolated_s > waypoints[-1, 0])] -= waypoints[-1, 0]
    interpolated_orientations = waypoints[index_absolute][:, 3] + (t * orientation_diffs)
    interpolated_orientations = (interpolated_orientations + np.pi) % (2 * np.pi) - np.pi
    interpolated_speeds = waypoints[index_absolute][:, 5] + (t * speed_diffs)
    
    reference = np.array([
        # Sort reference trajectory so the order of reference match the order of the states
        interpolated_positions[:, 0],
        interpolated_positions[:, 1],
        interpolated_speeds,
        interpolated_orientations,
        # Fill zeros to the rest so number of references mathc number of states (x[k] - ref[k])
        interpolated_s,
        np.zeros(len(interpolated_speeds)),
        np.zeros(len(interpolated_speeds))
    ])
    return reference
