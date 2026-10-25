# Sim to car

How to write code that runs unchanged in the AutoDRIVE sim and on the Jetson.

## The idea

Only a thin adapter layer knows which platform it is on. Everything above it (localization,
planning, control, safety) sees the same topics, frames and message types on both.

```text
shared stack (planner, controller, PF/EKF consumers, e-brake, mux, teleop)
        |  topic contract
sim adapter (sim_util)          car adapter (urg_node, vesc_*, ekf, particle_filter)
        |                               |
AutoDRIVE simulator             Hokuyo, VESC, joystick
```

## Topic contract

| Topic / frame | Type | Sim provides it with | Car provides it with |
| --- | --- | --- | --- |
| `/scan` | `LaserScan`, frame `laser` | `scan_remap_node` | `urg_node` |
| `/odom` | `Odometry` (wheel or sim odom) | `fake_odom_node` | `vesc_to_odom_node` |
| `/imu` (step 3) | `Imu` | remap of the AutoDRIVE IMU | remap of VESC `sensors/imu` |
| localization pose | `Odometry` | `/odom` (perfect) | `particle_filter` on `/pf/pose/odom` |
| `/drive/<source>` | `AckermannDriveStamped` | shared nodes | shared nodes |
| `/ackermann_cmd` | `AckermannDriveStamped` | `ackermann_converter_node` consumes it | `ackermann_to_vesc_node` consumes it |
| TF | `map -> odom -> base_link -> laser` | static `map->odom` + simulator | particle filter + EKF |

Step 3 renames the localization output to `/localization/odom` on both platforms, so planners
stop choosing between `/odom` and `/pf/pose/odom`. Until then, make the pose topic a parameter.

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
- If you change a contract topic name or type, change both adapters and every consumer in the
  same change, and update this file.

## Now vs after step 3

- Now: one branch. `PLATFORM=car|sim` (default `sim`, set in `watod-config.local.sh`) picks
  `<module>_bringup/config/car/` or `config/sim/` and which nodes start.
- After step 3: `watod` sets `PLATFORM=car|sim` (detected from `/etc/nv_tegra_release`), launch
  files load `config/car/` or `config/sim/`, and a CI contract test checks every topic above
  publishes at its expected rate.
