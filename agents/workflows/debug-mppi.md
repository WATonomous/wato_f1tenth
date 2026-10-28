# Debug MPPI

MPPI and the opponent predictor live in `src/action/`, off by default until step 4 (JAX image),
because MPPI still needs tuning and testing. They were ported to build on Jazzy but have not been run there.
Sources (copied from `mppi_test`): `docs/mppi/` (`IMPORTANT_DEBUG_MPPI.md`, `rodney_notes.md`,
`MPPI_README.md`, `MPPI_SYSTEM_OVERVIEW.md`, `mppi_commands.md`, `start_pf.sh`).

## Packages

| Package | What it is |
| --- | --- |
| `mppi_example` | JAX MPPI node (Python), port of mlab-upenn/mppi_example; 8192 samples x 8 steps at 40 Hz |
| `mppi_cpp` | C++/CUDA port with the same interface; node name `lmppi_node`; work in progress |
| `mppi_bringup` | Params, maps and launch files (e7, ICRA, IV) |
| `opponent_predictor` | Lidar clustering + Kalman filter along the raceline; `/opponent/predicted_path` |

MPPI publishes `/drive`, remapped to `/hal/drive/autonomy` in `mppi_cpp`'s `e7` launch file.

## Run

On `big_infra_update`: set `CONTROLLER=mppi` in `watod-config.local.sh`; `action.launch.yaml` then
starts `launch/controllers/mppi.launch.yaml`, which includes `mppi_cpp`'s `e7.launch.py` (same
params on both platforms for now; raceline and wall map from `race_tracks/tracks/<TRACK>/`).
Maps and racelines live only in `race_tracks` now. `mppi_bringup`'s launch files, `mppi_example`'s
`config.yaml` and `opponent_predictor`'s `waypoint_path` still name the deleted `waypoints/` and
`maps/` folders: point them at `race_tracks` when they are made to run (step 4). The opponent predictor is not wired
into a bringup yet; run it by hand in an `action:dev` container:

```bash
ros2 launch opponent_predictor e7_opp_pred.launch.py
```

On the old `mppi_test` branch (as last driven): `docker exec` into the robot dev container,
`ros2 launch mppi_cpp e7.launch.py`, and the predictor in a second terminal. Either way the
particle filter must run first; MPPI reads `/hal/localization/odom` (`odom_topic`).

## First checks

```bash
ros2 topic hz /drive                    # should match the control rate
ros2 topic hz /mppi/optimal_trajectory
ros2 param dump /lmppi_node | grep -E "opponent_cost_enabled|opponent_behavior_mode|opponent_cost_weight|opponent_cost_radius|opponent_follow_weight|opponent_pass_weight|opponent_auto"
ros2 topic echo /mppi/debug/opponent_active
ros2 topic echo /mppi/debug/cost_opponent_sum
ros2 topic echo /mppi/debug/cost_opponent_follow_sum
ros2 topic echo /mppi/debug/cost_opponent_pass_sum
ros2 topic echo /mppi/debug/min_opponent_dist
```

## Known behaviour and past fixes

- **When the control step fires.** It runs from the PF pose callback, rate-limited to the control
  rate, plus a watchdog (`control_watchdog_hz`, `control_watchdog_max_silence_sec`) in case the
  PF stalls. A pure timer in a single-threaded executor starved the pose callback.
- **Debug QoS.** Debug topics are BEST_EFFORT; RELIABLE debug publishers blocked the control
  loop when a remote subscriber lagged. Keep `/drive` RELIABLE.
- **Warm start reset.** Cleared when the time between control steps exceeds the horizon, or when
  the plan saturates: 5 or more of the first 6 actions at |a| >= `mppi_guard_aopt_threshold`
  (0.98) for `mppi_guard_saturation_callbacks` (4) ticks in a row.
- **Several subscribers to `/hal/localization/odom`** (MPPI plus the opponent predictor) can lower the PF
  rate. A dedicated republisher was proposed but not built.
- **Raceline format.** MPPI needs each waypoint's distance to the left and right wall.
- **Lidar detector.** `front_fov_only: true` is currently required for correct behaviour (cause
  not yet investigated).

## Performance on the Orin

- Power mode and clocks: `deploy-to-car.md`, step 5. Core pinning: step 9.
- JAX preallocates GPU memory (`XLA_PYTHON_CLIENT_PREALLOCATE=true`, `MEM_FRACTION=0.4`). On an
  8 GB Orin Nano Super, that memory is shared with the CPU.
- Python garbage collection is tuned to avoid 200-500 ms stalls; don't revert that casually.

## JAX environment (on `mppi_test`)

JAX 0.4.35 (dev build) is copied out of `dustynv/jax:r36.4.0` (Python 3.10, CUDA 12). cuDNN 9.4
is copied over the base image, and `numba==0.59.1` with `numpy==1.26.4` is pinned, because JAX and
numba fail to load otherwise. Step 4 rebuilds this for Python 3.12 on the Jazzy Jetson image.
