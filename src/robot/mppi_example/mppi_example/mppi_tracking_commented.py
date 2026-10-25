"""
MPPI (Model Predictive Path Integral) Controller — Annotated Version

This file implements the full MPPI algorithm from the lecture slides.
Every method maps to a step in the 5-step algorithm:

    1. Sample K noise perturbations
    2. Perturb the nominal control sequence
    3. Roll out each perturbed control through dynamics
    4. Compute trajectory rewards
    5. Compute importance weights and update the nominal

Jargon decoder:
    @partial(jax.jit, static_argnums=(0))
        = "compile this method, treat 'self' as unchanging"
        This is just the JAX way to jit a class method.
        partial(...) pre-fills the first argument of jax.jit.
        static_argnums=(0) means "self doesn't change between calls,
        so don't retrace when the same object calls this."
        You can mentally ignore this decorator — it just means "this runs fast."

    vmap(fn, in_axes=(0, None))
        = "call fn on each row of the first arg, broadcast the second arg"
        in_axes=(0, None) means: batch dim 0 of arg1, share arg2 across all calls.

    a_opt = nominal control sequence, shape (H, 2) — what we're trying to improve
    da    = noise perturbations, shape (K, H, 2) — random exploration around nominal
    K = n_samples, H = n_steps
"""
import jax
import jax.numpy as jnp
import os, sys
sys.path.append("../")
from functools import partial
import numpy as np


class MPPI():
    """An MPPI based planner."""
    def __init__(self, config, env, jrng,
                 temperature=0.01, damping=0.001, track=None):
        # ---- Store config ----
        self.config = config
        self.n_iterations = config.n_iterations  # how many MPPI refinement passes per timestep (usually 1)
        self.n_steps = config.n_steps            # H: horizon length (how many future steps to plan)
        self.n_samples = config.n_samples        # K: number of random trajectories to sample
        self.temperature = temperature           # lambda: controls greediness of weighting
                                                 #   low (0.01) = greedy, only best samples matter
                                                 #   high (1.0) = exploratory, all samples contribute
        self.damping = damping                   # prevents division by zero in weight normalization
        self.a_std = jnp.array(config.control_sample_std)  # noise std per control channel [steer, accel]
        self.a_cov_shift = config.a_cov_shift
        self.adaptive_covariance = (config.adaptive_covariance and self.n_iterations > 1) or self.a_cov_shift
        self.a_shape = config.control_dim        # = 2 (steering velocity, acceleration)
        self.env = env                           # dynamics model + reward functions (see infer_env.py)
        self.jrng = jrng                         # JAX random number generator
        self.init_state(self.env, self.a_shape)

        # Upper-triangular ones matrix for computing reward-to-go.
        # When you multiply this by a reward vector [r0, r1, r2, ...],
        # you get [r0+r1+r2+..., r1+r2+..., r2+..., ...] — the cumulative
        # future reward from each timestep onwards.
        self.accum_matrix = jnp.triu(jnp.ones((self.n_steps, self.n_steps)))
        self.track = track


    def init_state(self, env, a_shape):
        """Initialize the nominal control sequence to zeros."""
        dim_a = jnp.prod(a_shape)  # = 2
        self.env = env
        # a_opt: the nominal (best-so-far) control sequence, shape (H, 2)
        # Initialized to zeros = "do nothing"
        # The 0.0 * random is a hack to get the right JAX array type for vmap
        self.a_opt = 0.0*jax.random.uniform(self.jrng.new_key(), shape=(self.n_steps, dim_a))

        # Covariance for adaptive noise (optional, off by default)
        if self.a_cov_shift:
            self.a_cov = (self.a_std**2)*jnp.tile(jnp.eye(dim_a), (self.n_steps, 1, 1))
            self.a_cov_init = self.a_cov.copy()
        else:
            self.a_cov = None
            self.a_cov_init = self.a_cov


    def update(self, env_state, reference_traj):
        """
        Called once per control cycle (e.g., at 20-50 Hz from mppi_node.py).
        This is the entry point — the only method the ROS node calls.

        Args:
            env_state: current car state [x, y, steer, v, yaw, yaw_rate, slip] — shape (7,)
            reference_traj: target trajectory from waypoints — shape (H+1, 7)
        """
        # STEP 6 from the algorithm: shift the previous solution forward (warm start).
        # Drop u[0] (already executed), append zero at the end.
        self.a_opt, self.a_cov = self.shift_prev_opt(self.a_opt, self.a_cov)

        # Run the 5-step MPPI algorithm (usually once, but n_iterations > 1 = iterative refinement)
        for _ in range(self.n_iterations):
            self.a_opt, self.a_cov, self.states, self.traj_opt = self.iteration_step(
                self.a_opt, self.a_cov, self.jrng.new_key(), env_state, reference_traj
            )

        # Optional: convert cartesian (x,y) states to frenet (s, ey) for visualization
        if self.track is not None and self.config.state_predictor in self.config.cartesian_models:
            self.states = self.convert_cartesian_to_frenet_jax(self.states)
            self.traj_opt = self.convert_cartesian_to_frenet_jax(self.traj_opt)
        self.sampled_states = self.states

        # After this, mppi_node.py reads self.a_opt[0] to get the control to execute.


    @partial(jax.jit, static_argnums=(0))  # "compile this, self doesn't change"
    def shift_prev_opt(self, a_opt, a_cov):
        """
        Warm start: shift the nominal control sequence forward by 1 timestep.

        Before: [u0, u1, u2, ..., u_{H-1}]        (u0 was just executed)
        After:  [u1, u2, ..., u_{H-1}, u_{H-1}]    (repeat last = extrapolate the plan)

        This is why MPPI converges fast — we don't start from scratch each cycle,
        we reuse last cycle's solution shifted by one step.
        """
        a_opt = jnp.concatenate([
            a_opt[1:, :],   # drop first row
            a_opt[-1:, :]   # repeat last row instead of zero-padding
        ])  # shape stays (H, 2)

        if self.a_cov_shift:
            # Also shift covariance, reset last step to default
            a_cov = jnp.concatenate([
                a_cov[1:, :],
                jnp.expand_dims((self.a_std**2)*jnp.eye(self.a_shape), axis=0)
            ])
        else:
            a_cov = self.a_cov_init  # reset to initial (fixed covariance mode)
        return a_opt, a_cov


    @partial(jax.jit, static_argnums=(0))  # "compile this, self doesn't change"
    def iteration_step(self, a_opt, a_cov, rng_da, env_state, reference_traj):
        """
        ONE FULL MPPI UPDATE — this is the core algorithm (steps 1-5).

        Args:
            a_opt: current nominal controls, shape (H, 2)
            a_cov: noise covariance (None if not adaptive)
            rng_da: JAX random key for sampling noise
            env_state: current car state, shape (7,)
            reference_traj: target waypoints, shape (H+1, 7)

        Returns:
            a_opt: updated nominal controls
            a_cov: updated covariance (if adaptive)
            states: all K rolled-out trajectories, shape (K, H, 7) — for visualization
            traj_opt: the optimal trajectory, shape (H, 7) — for visualization
        """
        # Split one random key into 3 independent keys (JAX requires explicit key management)
        rng_da, rng_da_split1, rng_da_split2 = jax.random.split(rng_da, 3)

        # ===== STEP 1: SAMPLE K NOISE PERTURBATIONS =====
        # Shape: (K, H, 2) — K samples, each is a full (H, 2) noise sequence
        # truncated_normal: like normal() but clipped so that (a_opt + da) stays in [-1, 1]
        # The bounds ensure the perturbed controls don't exceed [-1, 1] after adding
        da = jax.random.truncated_normal(
            rng_da,
            -jnp.ones_like(a_opt) * self.a_std - a_opt,   # lower bound for noise
            jnp.ones_like(a_opt) * self.a_std - a_opt,    # upper bound for noise
            shape=(self.n_samples, self.n_steps, self.a_shape)
        )  # (K, H, 2)

        # ===== STEP 2: PERTURB NOMINAL =====
        # expand_dims adds a K dimension to a_opt: (H, 2) → (1, H, 2)
        # then broadcasting: (1, H, 2) + (K, H, 2) = (K, H, 2)
        # clip to [-1, 1] = actuator limits (controls are normalized)
        actions = jnp.clip(jnp.expand_dims(a_opt, axis=0) + da, -1.0, 1.0)  # (K, H, 2)

        # ===== STEP 3: ROLL OUT EACH PERTURBED CONTROL =====
        # vmap(rollout, in_axes=(0, None, None)) means:
        #   - actions: batch over axis 0 (each of K samples gets its own control sequence)
        #   - env_state: None = same starting state for ALL K rollouts
        #   - rng_key: None = same random key for all (deterministic dynamics)
        # Result: K trajectories, each of H states
        states = jax.vmap(self.rollout, in_axes=(0, None, None))(
            actions, env_state, rng_da_split1
        )  # (K, H, 7)

        # ===== STEP 4: COMPUTE TRAJECTORY COSTS =====
        # vmap(reward_fn, in_axes=(0, None)) means:
        #   - states: batch over axis 0 (each of K trajectories scored independently)
        #   - reference_traj: None = same reference for all K samples
        # Result: per-step reward for each sample
        if self.config.state_predictor in self.config.cartesian_models:
            reward = jax.vmap(self.env.reward_fn_xy, in_axes=(0, None))(
                states, reference_traj
            )  # (K, H) — one reward value per (sample, timestep)
        else:
            reward = jax.vmap(self.env.reward_fn_sey, in_axes=(0, None))(
                states, reference_traj
            )  # (K, H)

        # ===== STEP 5a: COMPUTE CUMULATIVE RETURNS (reward-to-go) =====
        # For each sample, convert per-step rewards [r0, r1, ..., r_{H-1}]
        # into reward-to-go [r0+r1+...+r_{H-1}, r1+...+r_{H-1}, ..., r_{H-1}]
        # This way early timesteps are weighted by the FULL future trajectory quality,
        # not just the immediate reward.
        R = jax.vmap(self.returns)(reward)  # (K, H)

        # ===== STEP 5b: COMPUTE IMPORTANCE WEIGHTS =====
        # For each timestep independently, compute softmax weights over K samples.
        # vmap(weights, in_axes=1, out_axes=1) means:
        #   "apply weights() to each COLUMN (timestep) of R"
        # So timestep t=0 gets its own softmax, t=1 gets its own, etc.
        w = jax.vmap(self.weights, 1, 1)(R)  # (K, H)

        # ===== STEP 5c: WEIGHTED AVERAGE OF NOISE → UPDATE NOMINAL =====
        # For each timestep, compute: da_opt[t] = sum_k( w[k,t] * da[k,t,:] )
        # vmap(jnp.average, (1, None, 1)) means:
        #   "for each timestep (axis 1 of da and w), compute weighted avg over samples (axis 0)"
        da_opt = jax.vmap(jnp.average, (1, None, 1))(da, 0, w)  # (H, 2)

        # Add the weighted noise to the nominal and clip to valid range
        a_opt = jnp.clip(a_opt + da_opt, -1.0, 1.0)  # (H, 2)

        # Optional: update noise covariance adaptively (MPOPI-style)
        # If enabled, the covariance shrinks toward directions that produced good samples
        if self.adaptive_covariance:
            # Compute outer product of each noise vector with itself
            a_cov = jax.vmap(jax.vmap(jnp.outer))(da, da)  # (K, H, 2, 2)
            # Weighted average of outer products = new covariance
            a_cov = jax.vmap(jnp.average, (1, None, 1))(a_cov, 0, w)  # (H, 2, 2)
            # Add small identity to prevent covariance from collapsing to zero
            a_cov = a_cov + jnp.eye(self.a_shape) * 0.00001

        # Compute the optimal trajectory for visualization
        if self.config.render:
            traj_opt = self.rollout(a_opt, env_state, rng_da_split2)  # (H, 7)
        else:
            traj_opt = states[0]  # just use first sample's trajectory as approximation

        return a_opt, a_cov, states, traj_opt


    @partial(jax.jit, static_argnums=(0))
    def returns(self, r):
        """
        Convert per-step rewards to reward-to-go (cumulative future reward).

        Input:  r = [r0, r1, r2, ..., r_{H-1}]
        Output: R = [r0+r1+...+r_{H-1}, r1+...+r_{H-1}, ..., r_{H-1}]

        This uses the upper-triangular matrix trick:
        [[1,1,1]    [r0]     [r0+r1+r2]
         [0,1,1]  @  [r1]  =  [r1+r2]
         [0,0,1]]    [r2]     [r2]
        """
        return jnp.dot(self.accum_matrix, r)  # (H,)


    @partial(jax.jit, static_argnums=(0))
    def weights(self, R):
        """
        Compute softmax importance weights from rewards.

        Input: R = reward-to-go for K samples at one timestep — shape (K,)
        Output: w = normalized weights — shape (K,), sums to 1

        High reward → high weight. The formula:
            w_k = exp( (R_k - max(R)) / ((max(R) - min(R)) + damping) / temperature )
            w_k = w_k / sum(w)

        The (R - max) / (max - min) part is numerical stability:
        - Subtracting max prevents exp() overflow
        - The best reward gets exponent 0; worse rewards get negative exponents
        - Dividing by range normalizes rewards before temperature scaling
        - damping prevents div by zero when all rewards are equal
        """
        R_stdzd = (R - jnp.max(R)) / ((jnp.max(R) - jnp.min(R)) + self.damping)
        w = jnp.exp(R_stdzd / self.temperature)
        w = w / jnp.sum(w)
        return w


    @partial(jax.jit, static_argnums=0)
    def rollout(self, actions, env_state, rng_key):
        """
        Simulate the car forward for H steps using given controls.

        Args:
            actions: control sequence, shape (H, 2) — [steering_vel, accel] per step
            env_state: starting state, shape (7,)
            rng_key: unused here (deterministic dynamics), passed for API compatibility

        Returns:
            states: trajectory, shape (H, 7) — car state at each future timestep

        This is a simple for-loop (not lax.scan). Since H=10 is small,
        JAX just unrolls it during compilation — 10 copies of the body.
        For larger H you'd want lax.scan, but for H=10 this is fine.
        """
        def rollout_step(env_state, actions, rng_key):
            actions = jnp.reshape(actions, self.env.a_shape)  # ensure shape is (2,)
            # env.step returns (next_state, variance, dynamics_diff)
            # We only need next_state
            (env_state, env_var, mb_dyna) = self.env.step(env_state, actions, rng_key)
            return env_state

        states = []
        for t in range(self.n_steps):
            env_state = rollout_step(env_state, actions[t, :], rng_key)
            states.append(env_state)

        return jnp.asarray(states)  # (H, 7)


    def convert_cartesian_to_frenet_jax(self, states):
        """
        Convert (x, y, yaw) states to Frenet frame (s, ey, epsi).
        Only used for visualization — the MPPI algorithm works in whatever
        frame the dynamics model uses.
        """
        states_shape = (*states.shape[:-1], 7)
        states = states.reshape(-1, states.shape[-1])
        converted_states = self.track.vmap_cartesian_to_frenet_jax(states[:, (0, 1, 4)])
        states_frenet = jnp.concatenate([converted_states[:, :2],
                                         states[:, 2:4] * jnp.cos(states[:, 6:7]),
                                         converted_states[:, 2:3],
                                         states[:, 2:4] * jnp.sin(states[:, 6:7])], axis=-1)
        return states_frenet.reshape(states_shape)
