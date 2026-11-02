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
3. Cross-track error `e`: positive when the car is right of the path.
4. Steering, clamped to ±`max_steering_angle`, with `v` the speed from `/hal/odom`:

   ```text
   δ = k_h · ψ + atan2(k_e · e, |v| + k_soft)
   ```

5. Speed: the closest waypoint's `z` (raceline speed), capped at `speed_limit`.

With the dead man switch released or no path, it publishes a zero command.

## Topics

| | Topic | Type |
| --- | --- | --- |
| in | `/global_planner/path` (latched) | `nav_msgs/Path` |
| in | `/hal/localization/odom` (pose) | `nav_msgs/Odometry` |
| in | `/hal/odom` (speed) | `nav_msgs/Odometry` |
| in | `/hal/dead_man_switch` | `std_msgs/Bool` |
| out | `/hal/drive/autonomy` | `ackermann_msgs/AckermannDriveStamped` |
| out | `/debug/stanley/markers` | `visualization_msgs/MarkerArray` |
| out | `/debug/stanley/path` (latched, once per path) | `visualization_msgs/MarkerArray` |
| out | `/debug/stanley/{cte, heading_error, heading_term, cte_term, delta_cmd}` | `std_msgs/Float32` |

The debug topics are published only when `enable_debug_vis` is true, at up to `debug_vis_rate_hz`.

## Parameters

In `action_bringup/config/{car,sim}/stanley_controller.yaml`:

| Parameter | Car | Sim | |
| --- | --- | --- | --- |
| `speed_limit_active` | true | true | apply `speed_limit` |
| `speed_limit` | 4.5 | 5.0 | m/s |
| `k_h` | 1.3 | 0.75 | heading gain |
| `k_e` | 0.9 | 0.2 | cross-track gain |
| `k_soft` | 0.5 | 0.5 | low-speed softener, m/s |
| `max_steering_angle` | 0.52 | 0.52 | rad |
| `wheelbase` | 0.324 | 0.324 | m |
| `closest_point_window` | 40 | 40 | waypoints |
| `closest_point_recovery_dist` | 2.0 | 2.0 | m |
| `control_rate_hz` | 50 | 20 | Hz |
| `enable_debug_vis` | true | true | |
| `debug_vis_rate_hz` | 20 | 20 | Hz |

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
| Weaves on straights | lower `k_h`, then `k_e` |
| Stays off the line | raise `k_e` |
| Twitchy at low speed | raise `k_soft` towards 1.0 |
| Closest point jumps across the track | lower `closest_point_window` |

| `control_rate_hz` | `k_h` | `k_e` | `speed_limit` |
| --- | --- | --- | --- |
| 20 | 0.85 | 0.7 | 4.0 |
| 50 | 1.3 | 0.9 | 4.5 |
