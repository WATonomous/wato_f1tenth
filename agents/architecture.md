# Architecture

How the repo is laid out, what runs where, and how data flows on `big_infra_update`
(ROS 2 Jazzy on the `wato_monorepo_template` infra). Step 3 of the migration adds platform
detection and the full topic contract; see the end of this file.

## Repo layout

```text
watod, watod-config.sh     docker compose wrapper and its documented defaults (personal: watod-config.local.sh)
watod_scripts/             env generation (.env), compose, test, bag, dev-env, build-base
modules/                   docker-compose.yaml (base) + dep, dev, test, watcloud overlays
docker/                    <module>.Dockerfile, template.Dockerfile (shared stages), base/ (base image layers)
src/<module>/              ROS 2 packages grouped by module; each module has <module>_bringup
src/wato_test/             Catch2 test helpers (add_wato_test)
docs/mppi/                 MPPI notes from mppi_test (run procedure, debugging, system overview)
agents/                    these docs
```

## Modules and containers

Each module is its own image and container (`<module>_bringup`). `ACTIVE_MODULES` picks which
start; `infrastructure` always starts. Append `:dev` for a dev container with `src/<module>`
mounted at `/ws/src/<module>`.

| Module | Packages | Runs |
| --- | --- | --- |
| `infrastructure` | `infrastructure_bringup`, `tf_static_relay`, `vision_msgs_markers`, `wato_lifecycle_manager` | Foxglove bridge (port `uid*20 + 2`), TF relays; builds `vesc_msgs` so Foxglove can decode it |
| `interfacing` | `vesc` (vesc_driver, vesc_ackermann, vesc_msgs), `teleop_utils`, `ackermann_mux`, `safety_system` (`safety_node`); RealSense driver (apt) | car: VESC, Hokuyo; both: joystick (USB or Bluetooth), mux, base_link->laser, RealSense IMU with `REALSENSE=true`; sim: keyboard teleop |
| `world_modeling` | `particle_filter` (+ range_libc), `ackermann_ekf`, `global_planner`, `costmap` | car: PF + map server; both: global planner; optional costmap, EKF, mapping, RViz |
| `action` | `pure_pursuit`, `mppi_example` (JAX), `mppi_cpp`, `mppi_bringup`, `opponent_predictor` | pure pursuit (default) or MPPI (`CONTROLLER=mppi`, untested) |
| `sim` | `sim_bringup`, `sim_util`, AutoDRIVE `autodrive_roboracer` (fetched at build) | AutoDRIVE bridge (port 4567) + sim adapter |
| `perception` | `perception_bringup` | nothing yet |

Bringup launch files read environment variables written to `modules/.env` by `watod`:

| Variable | Values | Effect |
| --- | --- | --- |
| `PLATFORM` | `sim` (default), `car` | which `config/<platform>/` YAML and which nodes start |
| `MAPPING` | `false` (default), `true` | slam_toolbox instead of localization; sim drops its static map->odom |
| `CONTROLLER` | `pure_pursuit` (default), `mppi`, `none` | controller in `action` |
| `REALSENSE` | `false` (default), `true` | RealSense driver in `interfacing`; IMU on `/camera/camera/imu` |
| `IMU` | `auto` (default), `realsense`, `vesc`, `none` | source of `/hal/imu` (auto: RealSense on the car with `REALSENSE=true`, AutoDRIVE in sim) |
| `LOCALIZATION` | `auto` (default), `pf`, `truth`, `none` | source of `/hal/localization/odom` (auto: `pf` on the car, `truth` in sim) |
| `GPU` | detected: `nvidia` or `none` | GPU or CPU base for `action`, `perception`, `world_modeling` |

Device access: `PLATFORM=car` adds `modules/docker-compose.car.yaml` (host `/dev`, privileged,
nvidia runtime); sim on Linux adds `modules/docker-compose.pc.yaml` (host `/dev/input` for a gamepad).

Images: `infrastructure`, `interfacing` and `sim` use the CPU base. `world_modeling`, `action` and
`perception` pick their base per architecture: x86 CPU or CUDA 12.9 + TensorRT, Jetson CUDA 12.8 +
TensorRT on arm64. All bases are built locally with `./watod build-base` for now.

RealSense: x86 uses the apt `librealsense2`. On arm64 the `interfacing` image rebuilds the same
version with the RSUSB (libusb) backend over it and holds the apt package, because JetPack kernels
lack the `hid_sensor` IIO drivers the kernel backend needs for the D435i IMU.

## Sim data flow

All shared topics are the `/hal/*` contract (`workflows/sim-to-car.md`).

```text
AutoDRIVE simulator (host) <-> autodrive_bridge (sim)   /autodrive/roboracer_1/*
  /autodrive/roboracer_1/lidar          -> scan_remap_node     -> /hal/scan
  /autodrive/roboracer_1/imu            -> imu_remap_node      -> /hal/imu
  /tf + /autodrive/roboracer_1/odom     -> fake_odom_node      -> /hal/odom, /hal/localization/odom (ground truth)
  static map -> odom                                           (stands in for the PF)
  /hal/drive/{ebrake,joystick,keyboard,autonomy} -> ackermann_mux -> /hal/ackermann_cmd
  /hal/ackermann_cmd                    -> ackermann_converter -> /autodrive/roboracer_1/{throttle,steering}_command
  raceline CSV -> global_planner -> /global_planner/path
  /global_planner/path + /hal/localization/odom + /hal/odom + /hal/dead_man_switch -> pure_pursuit -> /hal/drive/autonomy
```

## Car data flow (`PLATFORM=car`)

```text
Hokuyo (Ethernet 192.168.0.10) -> urg_node                     -> /hal/scan
VESC (/dev/sensors/vesc)       -> vesc_driver + vesc_to_odom   -> /hal/odom, sensors/imu
RealSense D435i (USB)          -> realsense2_camera            -> /hal/imu (IMU=realsense)
/hal/scan + /hal/odom + map    -> particle_filter (rmgpu)      -> /hal/localization/odom, TF map->odom
/joy                           -> joypad_node (gamepad_node)   -> /hal/drive/joystick, /hal/dead_man_switch
/hal/drive/*                   -> ackermann_mux                -> /hal/ackermann_cmd -> ackermann_to_vesc -> VESC
/global_planner/path + /hal/localization/odom + /hal/odom -> pure_pursuit -> /hal/drive/autonomy
```

The car config follows `mppi_test` (the car as last driven): PF on `/hal/odom`, no EKF, no e-brake.
Both packages are kept and can be switched on (`ekf:=true`, `ebrake:=true`); the mux keeps the
e-brake slot at priority 100.

Mux priorities: e-brake 100, joystick 50, keyboard and autonomy 25 (0.2 s timeouts), in
`interfacing_bringup/config/{car,sim}/mux.yaml`.

## TF tree

`map -> odom -> base_link -> laser` (and `base_link -> camera_link -> camera_*` with `REALSENSE=true`)

- `map -> odom`: static identity in sim; the particle filter on the car; slam_toolbox when mapping.
- `odom -> base_link`: the simulator in sim; the VESC / EKF path on the car.
- `base_link -> laser`: static, x 0.27 m, z 0.11 m, from `interfacing` on both.
- `base_link -> camera_link`: static, x 0.27 m, z 0.31 m (20 cm above the lidar), from
  `interfacing` when the RealSense runs. The driver adds `camera_imu_optical_frame` and friends
  under `camera_link` (`base_frame_id: link`, because the driver prefixes the camera name).

## Config locations

| What | Where |
| --- | --- |
| mux, VESC, Hokuyo | `interfacing_bringup/config/{car,sim}/` |
| PF, raceline, maps, RViz | `world_modeling_bringup/config/{car,sim}/`, `maps/`, `rviz/` |
| pure pursuit | `action_bringup/config/{car,sim}/pure_pursuit.yaml` |
| gamepad mapping | `teleop_utils/config/joypad_config.yaml` |
| MPPI | `mppi_cpp/config/params_e7.yaml`, `mppi_bringup/config/` |

## Still to come (step 3)

- `watod` detects the platform too (today `PLATFORM` is set by hand; `GPU` is already
  detected, and `PLATFORM=car` already adds `modules/docker-compose.car.yaml` for devices and
  `runtime: nvidia`).
- `./watod contract` (live check) and a bag replay test in `./watod test` for the `/hal/*`
  contract.

See `workflows/sim-to-car.md` for the contract rules.
