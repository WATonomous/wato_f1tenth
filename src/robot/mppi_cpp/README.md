# mppi_cpp

C++/CUDA port of `mppi_example` (JAX MPPI controller). Same ROS 2 interface, same
parameter YAML (`config/params_e7.yaml`), same raceline CSV / map yaml inputs.

```
ros2 launch mppi_cpp e7.launch.py            # mirrors mppi_bringup/launch/e7.launch.py
```

Layout

| File | What |
|---|---|
| `src/mppi_node.cpp` | ROS 2 node: params, odom_gate/timer trigger, pose-delta estimator, guards, `/drive`, debug + markers, stats line |
| `src/mppi_cuda.cu` | sample → rollout → reward → returns kernel, per-step softmax weights kernel, warm-start shift |
| `include/mppi_cpp/dynamics.cuh` | F1TENTH single-track (`dynamic_ST`) / kinematic (`kinematic_ST`) models, RK4 |
| `src/track.cpp` | raceline CSV, nearest point, reference trajectory + speed profile, per-step friction |
| `src/wall_sdf.cpp` | map yaml + PGM → Euclidean distance field for the wall cost |
| `test/` | A/B harness against the JAX code (`compare_with_jax.py`, needs jax) |

Differences from `mppi_example`

- Opponent behaviour is always `clear` (radial keep-out cost only). The
  follow / pass / auto-overtake state machine and its parameters
  (`opponent_behavior_mode`, `opponent_follow_*`, `opponent_pass_*`,
  `opponent_auto_*`) are gone; unknown YAML keys are ignored by rclcpp.
- `map_dir` / `map_info.txt` loading is gone: `wpt_path` is the raceline CSV
  (absolute, or relative to `mppi_cpp/data`). Raceline, map yaml and pgm live in
  `data/`, so the package has no dependency on `mppi_bringup` / `bringup_robot`.
- The unused `/mppi/ego_odom_for_opp` relay and the `opponent_auto_*` debug
  topics are gone. Everything else under `/mppi/debug/*` is unchanged.
