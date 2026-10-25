# Architecture

How the repo is laid out, what runs where, and how data flows. The "Today" sections describe
this branch before step 2 of the migration. The "Target" section says what step 2 and 3 change.

## Today: repo layout

```text
watod, watod-config.sh     docker compose wrapper and its config (personal overrides: watod-config.local.sh)
watod_scripts/             env generation (.env), dev-env setup, bash completion
modules/                   docker-compose.<module>.yaml for vis_tools, robot, sim, samples
docker/                    one Dockerfile per module (robot/, vis_tools/, sim/, samples/)
src/robot/                 our ROS 2 packages
src/wato_msgs/             message packages
config/                    VS Code sample settings and architecture diagrams
setup-linux.md, setup-windows.md, system-architecture.md, raceline-gen.md   human docs
```

## Today: containers

`ACTIVE_MODULES` picks compose files; `MODE_OF_OPERATION` picks the profile (`deploy` or `develop`).

| Service | Profile | What it does |
| --- | --- | --- |
| `robot` | deploy | Runs `ros2 launch bringup_robot robot.launch.py` (legacy: gym_vis + costmap) |
| `robot_dev` | develop | Idles (`tail -F`); `src/` is mounted at `/home/bolty/ament_ws/src`. You build and launch by hand. |
| `foxglove` | both | `foxglove_bridge` on port `uid * 20` (for example 20000 for uid 1000), not 8765 |
| `sim` | both | AutoDRIVE API bridge; the simulator itself runs natively on the host and connects on port 4567 |
| `samples` | both | Template examples from ASD training; not used |

The compose project name is `watod_$USER`, so containers are named like `watod_<user>-robot_dev-1`.
The default registry in `watod_scripts/watod-setup-docker-env.sh` still points at `wato_asd_training`.

## Today: packages on this branch

| Package | Lang | Role |
| --- | --- | --- |
| `bringup_robot` | launch + YAML | Launch files (`minimumEx`, `robot`, `global-planner`, `mapping`) and config |
| `sim_util` | C++ | Sim adapter: `scan_remap_node`, `fake_odom_node`, `ackermann_converter_node` |
| `ackermann_mux` | C++ | Picks one drive command by priority. Vendored 2021 fork of `twist_mux` |
| `teleop_utils` | C++ | `keyboard_converter` and `joypad_node`; joypad also publishes `/dead_man_switch` |
| `global_planner` | C++ | Loads a raceline CSV, publishes `/global_planner/path` and `/global_planner/vis` |
| `pure_persuit` | C++ | Pure pursuit controller (the spelling is the package name; keep it until step 2) |
| `costmap` | C++ | Occupancy grid from `/scan` on `/costmap` |
| `gym_vis` | C++ | Legacy f1tenth_gym visualizer; expects `/ego_racecar/odom`. Deleted in step 2. |
| `action` | Python | Empty template package. Deleted in step 2. |
| `wato_msgs/foxglove_msgs` | msgs | Empty, and shadows the upstream package. Deleted in step 2. |
| `wato_msgs/sample_msgs` | msgs | Template messages; unused |

Car-only packages live on the `orin` and `mppi_test` branches: `vesc*`, `transport_drivers`,
`particle_filter`, `ackermann_ekf`, `safety_node` (`ebreak`), and on `mppi_test` also `mppi_example`
(JAX), `mppi_cpp`, `mppi_bringup` and `opponent_predictor`.

## Today: sim data flow (`minimumEx.launch.py`)

```text
AutoDRIVE sim (host) <-> sim container  /autodrive/roboracer_1/*
  /autodrive/roboracer_1/lidar  -> scan_remap        -> /scan
  AutoDRIVE /tf and speed       -> fake_odom         -> /odom
  /drive/{ebreak,joystick,keyboard,autonomy} -> ackermann_mux -> /ackermann_cmd
  /ackermann_cmd                -> ackermann_converter -> /autodrive/roboracer_1/{throttle,steering}_command
  raceline CSV -> global_planner -> /global_planner/path
  /global_planner/path + /odom + /dead_man_switch -> pure_persuit -> /drive/autonomy
```

Mux priorities (`bringup_robot/config/sim/mux.yaml`): `ebreak` 50, `joystick`, `keyboard` and
`autonomy` 25, each with a 0.2 s timeout.

Known sim leak: `pure_persuit` reads speed from `/autodrive/roboracer_1/odom` (`speed_topic`).
That is a sim topic inside a shared node. Step 3 fixes this class of problem.

## Today: car data flow (`orin` branch, `jetson_pf.launch.py`)

```text
Hokuyo (Ethernet 192.168.0.10) -> urg_node        -> /scan
VESC (/dev/sensors/vesc)       -> vesc_driver + vesc_to_odom -> /odom, sensors/imu
/odom + IMU                    -> ackermann_ekf   -> /ekf/odom, TF
/scan + /ekf/odom + map        -> particle_filter -> /pf/pose/odom, TF map->odom
/scan                          -> ebreak (TTC)    -> /drive/ebreak
/joy                           -> joypad_node     -> /drive/joystick, /dead_man_switch
/drive/*                       -> ackermann_mux   -> ackermann_cmd -> ackermann_to_vesc -> VESC
```

`jetson_up.launch.py` is the same minus the particle filter, map server, RViz and global planner.
On `orin`, pure pursuit's car topics (`/pf/pose/odom`, `/ekf/odom`) were set by editing its C++
defaults. That is why sim and car code diverged.

## TF tree

`map -> odom -> base_link -> laser`

- `map -> odom`: a static identity transform in sim; the particle filter on the car.
- `odom -> base_link`: from the simulator in sim; from the EKF on the car.
- `base_link -> laser`: static, x 0.27 m, z 0.11 m, on both.

## Target (after steps 2 and 3)

- Template modules: `infrastructure` (Foxglove, healthchecker, lifecycle manager), `interfacing`
  (car drivers, teleop, mux, e-brake), `world_modeling` (particle filter, EKF, planner, costmap),
  `action` (pure pursuit, later MPPI), and `sim` (AutoDRIVE bridge and `sim_util`).
- ROS 2 Jazzy on Ubuntu 24.04. A CPU image for most modules, and a Jetson image with CUDA and
  TensorRT. PCs with an NVIDIA GPU use a CUDA image.
- A topic contract: `/scan`, `/odom`, `/imu`, `/localization/odom`, `/drive/*`, `/ackermann_cmd`,
  plus the TF tree above. Only the sim and car adapters know platform-specific topics.
- `watod` detects a Jetson (car), a PC with an NVIDIA GPU, or a PC without one, and picks the
  services and compose overlay. Config lives in `config/car/` and `config/sim/`.

See `workflows/sim-to-car.md` for the contract rules.
