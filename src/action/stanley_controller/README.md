# stanley_controller

Stanley path-tracking controller. Follows `/global_planner/path` using the heading and cross-track
error at the front axle; speed comes from the path. Select it with `CONTROLLER=stanley` and
`LOCAL_PLANNER=none`.

## Control law

Each tick (`control_rate_hz`):

1. Find the waypoint closest to the front axle (`wheelbase` ahead of the pose), searching
   `closest_point_window` waypoints ahead of the last one. Beyond `closest_point_recovery_dist`,
   search the whole path.
2. Heading error `ψ`: path heading (closest → next waypoint) minus yaw.
3. Cross-track error `e`: distance from the front axle to the path segment it is on, positive when
   the car is right of the path.
4. Steering, clamped to ±`max_steering_angle`, with `v` the speed from `/hal/odom`:

   ```text
   δ = k_h · ψ + atan2(k_e · e, |v| + k_soft)
   ```

5. Speed: the slowest waypoint `z` (raceline speed) within `|v| · speed_lookahead_s` metres ahead,
   so the car brakes before slow corners, capped at `speed_limit`.

With the dead man switch released, no path, or no pose for `pose_timeout_s`, it publishes a zero
command. If the pose goes stale while armed, the car stays stopped after localization comes back
until the dead man switch is released and pressed again.

## Topics

| | Topic | Type |
| --- | --- | --- |
| in | `/global_planner/path` (latched) | `nav_msgs/Path` |
| in | `/hal/localization/odom` (pose) | `nav_msgs/Odometry` |
| in | `/hal/odom` (speed) | `nav_msgs/Odometry` |
| in | `/hal/dead_man_switch` | `std_msgs/Bool` |
| in | `/local_planner/path` (`LOCAL_PLANNER` set) | `nav_msgs/Path`, base_link, open, speed in `z` |
| in | `/local_planner/active` (latched, `LOCAL_PLANNER` set) | `std_msgs/Bool` |
| out | `/hal/drive/autonomy` | `ackermann_msgs/AckermannDriveStamped` |
| out | `/debug/stanley/markers` | `visualization_msgs/MarkerArray` |
| out | `/debug/stanley/path` (latched, once per path) | `visualization_msgs/MarkerArray` |
| out | `/debug/stanley/{cte, heading_error, heading_term, cte_term, delta_cmd}` | `std_msgs/Float32` |

The debug topics are published only when `enable_debug_vis` is true, at up to `debug_vis_rate_hz`.

With `LOCAL_PLANNER` set (for example `frenet`), the launch file sets `use_local_planner`. Stanley then
follows `/local_planner/path` while `/local_planner/active` is true and the path is younger than
`local_path_timeout_s`, and the global path otherwise. The local path is transformed into the map
frame at its own timestamp (the latest transform if that time isn't buffered), read as an open path
(no wrap from its end to its start, and no speed look-ahead past its end), and searched whole every
tick. It logs `following the local path` / `following the global path` when it switches, and the
debug markers and text show which path it is following.

## Parameters

In `action_bringup/config/{car,sim}/stanley_controller.yaml`:

| Parameter | Car | Sim | |
| --- | --- | --- | --- |
| `speed_limit_active` | true | true | apply `speed_limit` |
| `speed_limit` | 10.0 | 5.0 | m/s |
| `speed_lookahead_s` | 0.4 | 0.0 | s of driving to look ahead for the speed (0 = closest waypoint) |
| `k_h` | 0.95 | 0.75 | heading gain |
| `k_e` | 0.7 | 0.2 | cross-track gain |
| `k_soft` | 0.5 | 0.5 | low-speed softener, m/s |
| `max_steering_angle` | 0.52 | 0.52 | rad |
| `wheelbase` | 0.324 | 0.324 | m |
| `closest_point_window` | 40 | 40 | waypoints |
| `closest_point_recovery_dist` | 2.0 | 2.0 | m |
| `pose_timeout_s` | 0.5 | 0.5 | s, stop without a fresh pose |
| `use_local_planner` | launch | launch | set from `LOCAL_PLANNER` by the launch file |
| `local_path_timeout_s` | 0.5 | 0.5 | s: an older local path falls back to the global path |
| `control_rate_hz` | 50 | 20 | Hz |
| `enable_debug_vis` | true | true | |
| `debug_vis_rate_hz` | 20 | 20 | Hz |

`speed_limit`, `speed_lookahead_s`, `k_h`, `k_e` and `control_rate_hz` have no default in code, so
the node won't start if one is missing from the YAML. `control_rate_hz`, `debug_vis_rate_hz`,
`closest_point_window`, `closest_point_recovery_dist` and `pose_timeout_s` must be greater than 0;
`speed_lookahead_s` must be 0 or more.

## Running

```bash
export CONTROLLER="stanley"     # watod-config.local.sh
export LOCAL_PLANNER="none"
./watod build && ./watod up
```

Arm inside a container or with the gamepad dead man button (R1) in car:

```bash
ros2 topic pub --qos-durability transient_local --once /hal/dead_man_switch std_msgs/msg/Bool "{data: true}"
```

To tune on the car without rebuilding images, run `action` as a dev container and relaunch after
each YAML change:

```bash
export ACTIVE_MODULES="interfacing world_modeling action:dev"   # watod-config.local.sh
./watod up && ./watod -t action_bringup_dev
colcon build --packages-select action_bringup stanley_controller && source install/setup.bash
ros2 launch action_bringup action.launch.yaml
```

The dev container doesn't start the controller itself; launch it again after a reboot.

## Tests

The control law is in `src/stanley.cpp` (no ROS node), with unit tests in `test/test_stanley.cpp`:

```bash
./watod test action stanley_controller                                    # on the host
colcon build --packages-up-to stanley_controller wato_test && colcon test --packages-select stanley_controller   # in the dev container
```

## Debug markers

| Marker | Meaning |
| --- | --- |
| grey line | path (on `/debug/stanley/path`) |
| yellow sphere | front axle |
| green sphere | closest waypoint |
| green arrow | path heading |
| orange arrow | car heading |
| CTE line | cyan/blue: right of path, yellow/red: left; greener = smaller |
| magenta arrow | steering command |

## Tuning

| Symptom | Change |
| --- | --- |
| Turns in late, runs wide | raise `k_h` or lower `speed_limit` |
| Enters slow corners too fast | raise `speed_lookahead_s` |
| Weaves on straights | lower `k_h`, then `k_e` (on the car, much of it comes from noise in the localization heading) |
| Stays off the line | raise `k_e` |
| Twitchy at low speed | raise `k_soft` towards 1.0 |
| Closest point jumps across the track | lower `closest_point_window` |

| `control_rate_hz` | `k_h` | `k_e` | `speed_limit` | `speed_lookahead_s` |
| --- | --- | --- | --- | --- |
| 20 | 0.85 | 0.7 | 4.0 | 0 |
| 50 | 1.3 | 0.9 | 4.5 | 0 |
| 50 | 1.0 | 0.9 | 6.0 | 0.4 |
| 50 | 0.95 | 0.85 | 10.0 | 0.4 |
| 50 | 0.95 | 0.7 | 5.0 | 0.4 |
| 50 | 0.95 | 0.7 | 10.0 | 0.4 |

At 9 m/s the car brakes hard into the e7 bottom-hallway corner, the nose dives, the lidar reads the
floor 3-4 m ahead and the particle filter falls 1.7-2.9 m behind, so Stanley turns in late and hits
the wall (2026-11-10 and 11). The car was capped at 5 m/s for a day; it is back at 10 to test the EKF
and particle filter fixes (`localization-fix`). If those don't hold, cap it at 5 again, or use the
last resort: `max_decel` 2.5 in `vesc.yaml` with `TRACK=e7_fifth_v3-fixed`.
