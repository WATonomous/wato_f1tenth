# Sim to car

How to write code that runs unchanged in the AutoDRIVE sim and on the Jetson.

## The idea

Only a thin adapter layer knows which platform it is on. Everything above it (localization,
planning, control, safety) sees the same topics, frames and message types on both.

```text
shared stack (planner, controller, localization consumers, e-brake, mux, teleop)
        |  /hal/* topic contract
sim adapter (sim_util)          car adapter (urg_node, vesc_*, RealSense, particle_filter)
        |                               |
AutoDRIVE simulator             Hokuyo, VESC, RealSense, joystick
```

## Topic contract

Every topic the adapters provide or consume, and every drive input, lives under `/hal/`, so
`ros2 topic list | grep /hal` shows the whole contract. Drivers publish straight onto these names
through launch remaps; the old names (`/scan`, `/odom`, `/pf/pose/odom`, `/ackermann_cmd`,
`/drive/*`, `/dead_man_switch`) are gone.

| Topic | Type | Sim provides it with | Car provides it with |
| --- | --- | --- | --- |
| `/hal/scan` | `LaserScan`, frame `laser` | `scan_remap_node` | `urg_node` |
| `/hal/odom` | `Odometry`, `odom -> base_link` (wheel, or perfect sim odom) | `fake_odom_node` | `vesc_to_odom_node` |
| `/hal/imu` | `Imu` | `imu_remap_node` (frame `imu_link`) | RealSense (`IMU=realsense`, default with `REALSENSE=true`) or VESC (`IMU=vesc`, not working as intended yet) |
| `/hal/localization/odom` | `Odometry`, frame `map` | `fake_odom_node` ground truth (`LOCALIZATION=truth`, default) or `particle_filter` (`pf`, needs a sim map) | `particle_filter` |
| `/hal/battery` | `BatteryState` (drive pack: voltage, %, health) | none (AutoDRIVE has no battery) | `vesc_battery` from VESC `/sensors/core` |
| `/hal/drive/{autonomy,joystick,keyboard,ebrake}` | `AckermannDriveStamped` | shared nodes | shared nodes |
| `/hal/dead_man_switch` | `Bool` (transient local) | gamepad teleop | gamepad teleop |
| `/hal/ackermann_cmd` | `AckermannDriveStamped` (mux output) | `ackermann_converter_node` drives AutoDRIVE | `ackermann_to_vesc_node` drives the VESC |
| TF | `map -> odom -> base_link -> laser` | static `map->odom` + `fake_odom` | particle filter + VESC odometry |

Planners and controllers read the pose from `/hal/localization/odom` and the speed from
`/hal/odom`. Internal and debug topics (`/global_planner/path`, `/pf/viz/*`, `/camera/camera/*`,
`/autodrive/*`, VESC `sensors/*`) keep their own names.

The VESC IMU (`sensors/imu/raw`) has no `frame_id` and its readings haven't been checked. Wire
`IMU=vesc` only to debug it.

## What may differ between sim and car

Only YAML values, never code:

- Gains and limits (lookahead, speed limit, steering gain).
- Safety thresholds (commit `26a774d` on `orin` moved the e-brake TTC from 0.39 s to about 0.10 s for the car).
- Map and raceline file paths.
- `use_sim_time` (true only if the sim publishes `/clock`; always false on the car).

## Rules

- No shared node subscribes to `/autodrive/...`. If you need something from the sim, add it to
  `sim_util` and republish it on a contract topic.
- No `if sim:` branches in node code. Use a parameter whose value differs per platform YAML.
- The sim's odometry is perfect; the car's drifts. Don't tune against sim odometry and expect the
  same behaviour on the car.
- A new hardware-facing topic or drive input goes under `/hal/`, never outside it.
- If you change a contract topic name or type, change both adapters and every consumer in the
  same change, and update this file.

## Now vs after step 3

- Now: one branch. `PLATFORM=car|sim` (default `sim`, set in `watod-config.local.sh`) picks
  `<module>_bringup/config/car/` or `config/sim/` and which nodes start.
- Rest of step 3: `watod` detects `PLATFORM` (from `/etc/nv_tegra_release`).

## Checking the contract

The contract lives in `src/infrastructure/hal_contract/config/contract.yaml`: each topic's type,
minimum rate per platform, frame and required endpoints, the TF chain, and the old names that must
be gone. Two things check it:

- `./watod contract` checks the running stack (car or sim) and prints PASS / WARN / FAIL per
  topic. Topics whose source is off (`IMU=none`, `LOCALIZATION=none`, mapping) are skipped. An old
  name used only by `foxglove_bridge` is a warning: update that Foxglove layout.
- `./watod test sim sim_bringup` replays `sim_bringup/test/data/autodrive_contract` through
  `sim.launch.yaml` and checks the sensor topics, so a broken sim adapter fails in tests.

Change the contract file in the same change as the topics it describes.
