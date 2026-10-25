# mppi_example

ROS 2 package for the JAX MPPI controller.

Start with [`../../MPPI_GUIDE.md`](../../MPPI_GUIDE.md) for the math, code map, and tuning notes. The main runtime path is:

1. `mppi_node.py` receives odometry and publishes `/drive`.
2. `infer_env.py` builds the reference trajectory and wraps the dynamics/reward functions.
3. `mppi_tracking.py` samples controls, rolls out trajectories with JAX, weights rewards, and updates the nominal control sequence.
4. `dynamics_models/dynamics_models_jax.py` contains the vehicle models used during rollout.

[Original MPPI demo](https://youtube.com/watch?v=etC4URGOjhg)
