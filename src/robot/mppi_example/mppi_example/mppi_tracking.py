import jax
import jax.numpy as jnp
from functools import partial


class MPPI():
    """An MPPI based planner."""
    def __init__(self, config, env, jrng, 
                 temperature=0.01, damping=0.001, track=None):
        self.config = config
        self.n_iterations = config.n_iterations
        self.n_steps = config.n_steps
        self.n_samples = config.n_samples
        self.temperature = getattr(config, 'temperature', temperature)
        self.damping = getattr(config, 'damping', damping)
        self.a_std = jnp.array(config.control_sample_std)
        self.a_cov_shift = config.a_cov_shift
        self.adaptive_covariance = (config.adaptive_covariance and self.n_iterations > 1) or self.a_cov_shift
        self.a_shape = config.control_dim
        self.env = env
        self.jrng = jrng
        self.init_state(self.env, self.a_shape)
        self.accum_matrix = jnp.triu(jnp.ones((self.n_steps, self.n_steps)))
        self.track = track


    def init_state(self, env, a_shape):
        # uses random as a hack to support vmap
        # we should find a non-hack approach to initializing the state
        dim_a = jnp.prod(a_shape)  # np.int32
        self.env = env
        self.a_opt = 0.0*jax.random.uniform(self.jrng.new_key(), shape=(self.n_steps,
                                                dim_a))  # [n_steps, dim_a]
        # a_cov: [n_steps, dim_a, dim_a]
        if self.a_cov_shift:
            # note: should probably store factorized cov,
            # e.g. cholesky, for faster sampling
            self.a_cov = (self.a_std**2)*jnp.tile(jnp.eye(dim_a), (self.n_steps, 1, 1))
            self.a_cov_init = self.a_cov.copy()
        else:
            self.a_cov = None
            self.a_cov_init = self.a_cov
            
            
    def update(self, env_state, reference_traj, opponent_traj=None, opponent_active=False,
               reference_frictions=None):
        """reference_frictions: optional jnp.array of shape (n_steps,) giving the
        friction value at each horizon step. If None, the scalar config.friction
        is broadcast across all steps (legacy behavior)."""
        if opponent_traj is None:
            opponent_traj = jnp.zeros((self.n_steps, 2), dtype=jnp.float32)
        a_std, temperature, damping, reward_weights, cost_params, norm_params, friction = self.runtime_params(
            opponent_active=opponent_active
        )
        # friction from runtime_params is now a (n_steps,) array — see below.
        # If caller supplied per-step frictions (from the track waypoints), use those.
        if reference_frictions is not None:
            rf = jnp.asarray(reference_frictions, dtype=jnp.float32)
            # Defensive: pad/truncate to n_steps so the JIT trace shape is fixed.
            if rf.shape[0] != self.n_steps:
                if rf.shape[0] > self.n_steps:
                    rf = rf[:self.n_steps]
                else:
                    pad = jnp.full((self.n_steps - rf.shape[0],), rf[-1] if rf.shape[0] > 0 else self.config.friction,
                                   dtype=jnp.float32)
                    rf = jnp.concatenate([rf, pad])
            friction = rf
        self.a_opt, self.a_cov = self.shift_prev_opt(self.a_opt, self.a_cov, a_std)
        for _ in range(self.n_iterations):
            self.a_opt, self.a_cov, self.states, self.traj_opt = self.iteration_step(
                self.a_opt,
                self.a_cov,
                self.jrng.new_key(),
                env_state,
                reference_traj,
                a_std,
                temperature,
                damping,
                reward_weights,
                cost_params,
                norm_params,
                friction,
                opponent_traj,
            )
        
        if self.track is not None and self.config.state_predictor in self.config.cartesian_models:
            self.states = self.convert_cartesian_to_frenet_jax(self.states)
            self.traj_opt = self.convert_cartesian_to_frenet_jax(self.traj_opt)
        self.sampled_states = self.states


    def runtime_params(self, opponent_active=False):
        reward_weights = [
            getattr(self.config, 'xy_reward_weight', 1.0),
            getattr(self.config, 'velocity_reward_weight', 0.0),
            getattr(self.config, 'yaw_reward_weight', 0.0),
        ]
        opponent_enabled = (
            getattr(self.config, 'opponent_cost_enabled', False)
            and opponent_active
        )
        cost_params = [
            getattr(self.config, 'wall_cost_weight', 0.0) if getattr(self.config, 'wall_cost_enabled', False) else 0.0,
            getattr(self.config, 'wall_cost_margin', 0.0),
            getattr(self.config, 'wall_cost_power', 0.01),
            getattr(self.config, 'slip_cost_weight', 0.0) if getattr(self.config, 'slip_cost_enabled', False) else 0.0,
            getattr(self.config, 'slip_cost_beta_safe', 0.0),
            getattr(self.config, 'latacc_cost_weight', 0.0) if getattr(self.config, 'latacc_cost_enabled', False) else 0.0,
            getattr(self.config, 'latacc_cost_safe', 0.0),
            getattr(self.config, 'steer_sat_cost_weight', 0.0) if getattr(self.config, 'steer_sat_cost_enabled', False) else 0.0,
            getattr(self.config, 'steer_sat_soft_ratio', 0.0) * getattr(self.config, 'max_steering_angle', 0.0),
            getattr(self.config, 'opponent_cost_weight', 0.0) if opponent_enabled else 0.0,
            getattr(self.config, 'opponent_cost_radius', 0.8),
            getattr(self.config, 'opponent_cost_power', 2.0),
            getattr(self.config, 'opponent_cost_discount', 1.0),
            getattr(self.config, 'opponent_behavior_mode_id', 0.0),
            # Gate follow/pass weights by opponent_enabled too — otherwise the
            # zero-init opponent_xy_horizon (all (0, 0)) is treated as a
            # stationary opponent at map origin and MPPI swerves to "follow"
            # or "pass" it whenever no real opponent is active.
            getattr(self.config, 'opponent_follow_weight', 0.0) if opponent_enabled else 0.0,
            getattr(self.config, 'opponent_follow_distance', 1.2),
            getattr(self.config, 'opponent_same_lane_width', 0.7),
            getattr(self.config, 'opponent_pass_weight', 0.0) if opponent_enabled else 0.0,
            getattr(self.config, 'opponent_pass_lateral_offset', 0.55),
            getattr(self.config, 'opponent_pass_longitudinal_window', 1.5),
        ]
        return (
            jnp.asarray(self.config.control_sample_std, dtype=jnp.float32),
            jnp.asarray(self.temperature, dtype=jnp.float32),
            jnp.asarray(self.damping, dtype=jnp.float32),
            jnp.asarray(reward_weights, dtype=jnp.float32),
            jnp.asarray(cost_params, dtype=jnp.float32),
            jnp.asarray(self.config.norm_params, dtype=jnp.float32),
            # Friction defaults to a (n_steps,) array broadcast of config.friction.
            # `MPPI.update` overrides this with per-step values when reference_frictions is supplied.
            jnp.full((self.n_steps,), float(self.config.friction), dtype=jnp.float32),
        )

    
    @partial(jax.jit, static_argnums=(0))
    def shift_prev_opt(self, a_opt, a_cov, a_std):
        a_opt = jnp.concatenate([a_opt[1:, :],
                                a_opt[-1:, :]])  # [n_steps, a_shape]
        if self.a_cov_shift:
            a_cov = jnp.concatenate([a_cov[1:, :],
                                    jnp.expand_dims((a_std**2)*jnp.eye(self.a_shape),
                                                    axis=0)])
        else:
            a_cov = self.a_cov_init
        return a_opt, a_cov
    
    
    @partial(jax.jit, static_argnums=(0))
    def iteration_step(self, a_opt, a_cov, rng_da, env_state, reference_traj,
                       a_std, temperature, damping, reward_weights, cost_params, norm_params, friction,
                       opponent_traj):
        rng_da, rng_da_split1, rng_da_split2 = jax.random.split(rng_da, 3)
        da = jax.random.truncated_normal(
            rng_da,
            -jnp.ones_like(a_opt) * a_std - a_opt,
            jnp.ones_like(a_opt) * a_std - a_opt,
            shape=(self.n_samples, self.n_steps, self.a_shape)
        )  # [n_samples, n_steps, dim_a]

        actions = jnp.clip(jnp.expand_dims(a_opt, axis=0) + da, -1.0, 1.0)
        states = jax.vmap(self.rollout, in_axes=(0, None, None, None, None))(
            actions, env_state, rng_da_split1, norm_params, friction
        )
        
        if self.config.state_predictor in self.config.cartesian_models:
            reward = jax.vmap(self.env.reward_fn_xy, in_axes=(0, None, None, None, None))(
                states, reference_traj, reward_weights, cost_params, opponent_traj
            )
        else:
            reward = jax.vmap(self.env.reward_fn_sey, in_axes=(0, None, None))(
                states, reference_traj, reward_weights
            ) # [n_samples, n_steps]          
        reward = jnp.nan_to_num(reward, nan=-1e6, posinf=-1e6, neginf=-1e6)
        R = jax.vmap(self.returns)(reward) # [n_samples, n_steps], pylint: disable=invalid-name
        R = jnp.nan_to_num(R, nan=-1e6, posinf=-1e6, neginf=-1e6)
        w = jax.vmap(self.weights, (1, None, None), 1)(R, temperature, damping)  # [n_samples, n_steps]
        da_opt = jax.vmap(jnp.average, (1, None, 1))(da, 0, w)  # [n_steps, dim_a]
        a_opt = jnp.clip(a_opt + da_opt, -1.0, 1.0)  # [n_steps, dim_a]
        if self.adaptive_covariance:
            a_cov = jax.vmap(jax.vmap(jnp.outer))(
                da, da
            )  # [n_samples, n_steps, a_shape, a_shape]
            a_cov = jax.vmap(jnp.average, (1, None, 1))(
                a_cov, 0, w
            )  # a_cov: [n_steps, a_shape, a_shape]
            a_cov = a_cov + jnp.eye(self.a_shape)*0.00001 # prevent loss of rank when one sample is heavily weighted
            
        if self.config.render:
            traj_opt = self.rollout(a_opt, env_state, rng_da_split2, norm_params, friction)
        else:
            traj_opt = states[0]
            
        return a_opt, a_cov, states, traj_opt

   
    @partial(jax.jit, static_argnums=(0))
    def returns(self, r):
        # r: [n_steps]
        return jnp.dot(self.accum_matrix, r)  # R: [n_steps]


    @partial(jax.jit, static_argnums=(0))
    def weights(self, R, temperature, damping):  # pylint: disable=invalid-name
        # R: [n_samples]
        # R_stdzd = (R - jnp.min(R)) / ((jnp.max(R) - jnp.min(R)) + self.damping)
        # R_stdzd = R - jnp.max(R) # [n_samples] np.float32
        R_max = jnp.max(R)
        R_min = jnp.min(R)
        denom = jnp.maximum((R_max - R_min) + damping, 1e-6)
        R_stdzd = (R - R_max) / denom  # pylint: disable=invalid-name
        w = jnp.exp(R_stdzd / temperature)  # [n_samples] np.float32
        w_sum = jnp.sum(w)
        uniform = jnp.ones_like(w) / w.shape[0]
        w = jnp.where(w_sum > 0.0, w / w_sum, uniform)  # [n_samples] np.float32
        return w
    
    
    @partial(jax.jit, static_argnums=0)
    def rollout(self, actions, env_state, rng_key, norm_params, friction):
        """
        actions: [n_steps, a_shape]
        friction: (n_steps,) per-step friction (or scalar — see fallback below)
        env: {.step(states, actions), .reward(states)}
        env_state: np.float32
        actions: # a_0, ..., a_{n_steps}. [n_steps, a_shape]
        states: # s_1, ..., s_{n_steps+1}. [n_steps, env_state_shape]
        """
        # Ensure friction is a (n_steps,) array — scalars (legacy callers) get
        # broadcast. Doing this once outside the loop keeps the JIT trace stable.
        friction_arr = jnp.broadcast_to(jnp.atleast_1d(friction), (self.n_steps,))

        def rollout_step(env_state, action, mu_t, rng_key):
            action = jnp.reshape(action, self.env.a_shape)
            (env_state, env_var, mb_dyna) = self.env.step(
                env_state, action, rng_key, norm_params, mu_t
            )
            return env_state

        states = []
        for t in range(self.n_steps):
            env_state = rollout_step(env_state, actions[t, :], friction_arr[t], rng_key)
            states.append(env_state)

        return jnp.asarray(states)
    
    
    # @partial(jax.jit, static_argnums=(0))
    def convert_cartesian_to_frenet_jax(self, states):
        states_shape = (*states.shape[:-1], 7)
        states = states.reshape(-1, states.shape[-1])
        converted_states = self.track.vmap_cartesian_to_frenet_jax(states[:, (0, 1, 4)])
        states_frenet = jnp.concatenate([converted_states[:, :2], 
                                         states[:, 2:4] * jnp.cos(states[:, 6:7]),
                                         converted_states[:, 2:3],
                                         states[:, 2:4] * jnp.sin(states[:, 6:7])], axis=-1)
        return states_frenet.reshape(states_shape)
