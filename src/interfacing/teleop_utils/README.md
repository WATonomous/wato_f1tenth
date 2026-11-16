# teleop_utils: driving the car with a gamepad (and the keyboard in sim)

Two nodes:

- `joypad_node`, launched as `gamepad_node`: reads `/joy` from `joy_node` and publishes the manual
  drive command, the autonomy dead man switch, the gamepad mode and the gamepad state.
- `keyboard_converter`: turns `/cmd_vel` (the Foxglove teleop panel) into a drive command. Sim only.

Both start from `interfacing_bringup/launch/interfacing.launch.yaml`, on the car and on a PC
(`joystick:=true` by default; `keyboard` is on in sim). Settings: `config/joypad_config.yaml`.

## Connecting a pad

Tested with a PS4 DualShock 4; a PS5 DualSense uses the same SDL layout.

- **Bluetooth, car or PC:** on the host (not in a container), `./watod controller pair` with the
  pad in pairing mode (hold PS + Share, or PS + Create on a DualSense, until the light flashes). It
  scans, drops any stale pairing, pairs, trusts and connects, and saves the MAC in
  `watod-config.local.sh`. Later: `./watod controller connect`, `status` (adapter, connection,
  `/dev/input/js*`) or `forget`. On the Jetson (BlueZ 5.64) use this rather than the desktop
  Bluetooth settings; it runs `bluetoothctl` on the host, so it also works over SSH.
- **USB:** plug it in. On a PC, the `docker-compose.pc.yaml` overlay gives `interfacing` the host's
  `/dev/input`, so pads that connect after the container starts still show up.

Check: `ros2 topic echo /joy` (in the interfacing container) changes when you press things.

## Controls

All buttons are **toggles**: press once for on, again for off. `joy_node` runs with
`sticky_buttons: true` and keeps the state, so restarting `joy_node` (or the stack) turns
everything off. The Foxglove Drive tab shows every toggle; so does `ros2 topic echo /control/teleop`.

| Input | Does |
| --- | --- |
| **X** (cross) | Manual control on/off. On: the pad drives and beats autonomy in the mux, and the dead man is released |
| **R1** | Autonomy dead man on/off. On (with X off): the controller may drive |
| **R2** | Throttle (trigger travel through the curve below) |
| **L2** | Brake: subtracts from the throttle; in reverse, cancels it |
| **Left stick X** | Steering: stick x `steering_gain` (0.5236 rad = 30 degrees at full lock) |
| **O** (circle) | Reverse on/off. On: throttle past `reverse_throttle_threshold` (20%) reverses at `reverse_speed` (0.75 m/s) |
| **Square** | Pit limit on/off: forward speed capped at 1 m/s |
| Triangle, L1, PS | Unused |

### Handing over between manual and autonomy

1. Autonomy driving: R1 on, X off. Mode `AUTONOMY`.
2. Take over: press **X**. Mode `MANUAL`; autonomy is blocked from now on.
3. Give it back: press **X** again (the node sends one zero command, then goes quiet), then press
   **R1 twice** (off, on). Until then the mode is `BLOCKED` and the car stays stopped.

Step 3 is on purpose: using X to stop the car and then releasing it must never let autonomy drive
away by itself. To stop autonomy without taking manual control, toggle R1 off.

Modes on `/control/mode` (latched, sent on change): `MANUAL`, `AUTONOMY`, `BLOCKED`, `IDLE`,
`NO_GAMEPAD`. `drive_status` folds this into the one-word `/control/status`.

### Throttle, brake and reverse

The throttle curve is exponential, so the first half of the trigger is gentle:
`throttle = exp(4 (travel - 1)) - 0.0183`, times `max_speed` (10 m/s):

| Trigger travel | 25% | 50% | 75% | 90% | 100% |
| --- | --- | --- | --- | --- | --- |
| Speed asked (m/s) | 0.3 | 1.2 | 3.5 | 6.5 | 9.8 |

- **L2 goes through the same curve and is subtracted**, floored at zero: full R2 with 3/4 L2 asks for
  about 6.3 m/s. Letting go of R2 already asks for 0; the actual slowing down is done by the speed
  limiter in `ackermann_to_vesc` (4.0 m/s^2) and the VESC's braking, so L2 matters only while R2
  is still pressed.
- **Reverse** is one fixed speed, not proportional: below about 0.5 m/s the sensorless motor start
  stutters. Toggle O off to drive forward again. The pit limit does not apply in reverse.
- The VESC side caps reverse at 5 m/s anyway (`speed_min` in `interfacing_bringup/config/car/vesc.yaml`).

### Where else the command is shaped

```text
/joy -> gamepad_node -> /hal/drive/joystick (priority 50) -> ackermann_mux -> /hal/ackermann_cmd
                                                                -> ackermann_to_vesc -> VESC
```

- `ackermann_mux` (`interfacing_bringup/config/{car,sim}/mux.yaml`): joystick 50 beats keyboard and
  autonomy (25); each input times out after 0.2 s. The gamepad publishes only while X is on, so
  the mux falls through to autonomy when manual is off.
- `ackermann_to_vesc` (car): speeding up at most 3.3 m/s^2, slowing at 4.0 m/s^2, and a stop if no
  command arrives for 0.5 s. `/control/speed_limiter` shows asked vs limited speed.
- Top speed: `max_speed` here, `speed_max` in `vesc.yaml` (46200 ERPM = 10 m/s) and the VESC's own
  Max ERPM (`agents/workflows/vesc-firmware.md`). The lowest one wins.

### Pad lost

If `/joy` goes quiet for `joy_timeout_s` (0.5 s), for example the pad disconnects or its battery
dies: the mode becomes `NO_GAMEPAD` and the dead man is released, so autonomy stops. A manual
command stops arriving too, so the mux times out and `ackermann_to_vesc` brakes to a stop.

## Topics

| Topic | Type | QoS | What |
| --- | --- | --- | --- |
| `/joy` (in) | `sensor_msgs/Joy` | default | from `joy_node` (deadzone 0.08) |
| `/hal/drive/joystick` | `ackermann_msgs/AckermannDriveStamped` | reliable | manual command, only while X is on |
| `/hal/dead_man_switch` | `std_msgs/Bool` | reliable, latched | autonomy may drive |
| `/control/mode` | `std_msgs/String` | reliable, latched | gamepad mode (above) |
| `/control/teleop` | `f1tenth_msgs/TeleopState` | best effort, 10 Hz | toggles, trigger travel, steering, asked speed, connected |
| `/cmd_vel` (in, sim) | `geometry_msgs/Twist` | default | Foxglove teleop panel |
| `/hal/drive/keyboard` | `ackermann_msgs/AckermannDriveStamped` | default | keyboard command (priority 25) |

Keyboard (sim): `linear.x` is the speed (clamped to -3.5 to 20 m/s); with `linear.x` zero,
`angular.z` turns at 0.5 m/s with steering `pi/10 x angular.z` (at most 30 degrees).

## Settings (`config/joypad_config.yaml`)

The file is keyed `/**:` so it applies whatever the node is named (rule 6). If it does not load,
the node falls back to compiled-in defaults that differ (4 m/s, brake on the right stick, reverse
on L1): **reverse on the wrong button means the YAML was ignored.**

| Key | Value | Meaning |
| --- | --- | --- |
| `max_speed` | 10.0 | m/s at full throttle |
| `steering_gain` | 0.5236 | rad at full stick |
| `max_steering_rate` | 3.2 | rad/s, sent as `steering_angle_velocity` |
| `reverse_speed` | 0.75 | m/s while reversing (keep >= 0.5) |
| `reverse_throttle_threshold` | 0.2 | trigger travel that counts as pressed in reverse |
| `steering_axis`, `throttel_axis`, `break_axis` | 0, 5, 4 | SDL axes: left X, R2, L2 (key names are spelled as in the code) |
| `safety_button`, `autonomy_button`, `reverse_button`, `pit_limit_button` | 0, 10, 1, 2 | SDL buttons: X, R1, O, square |
| `joy_timeout_s` | 0.5 | release the dead man after this long without `/joy` |
| topic keys | | `gamepad_topic`, `dead_man_topic`, `mode_topic`, `teleop_state_topic`, `joy_topic` |

SDL game-controller layout (what `joy_node` reports): axes 0 left X, 1 left Y, 2 right X, 3 right Y,
4 L2, 5 R2; buttons 0 cross, 1 circle, 2 square, 3 triangle, 5 PS, 9 L1, 10 R1. Triggers read 1.0
released and -1.0 pressed, but **exactly 0.0 until first pressed**; the node counts that as
released, so a fresh pad does not start at half throttle.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Nothing happens | `./watod controller status`; `/dev/input/js0` exists; `ros2 topic echo /joy` moves; mode is not `NO_GAMEPAD` |
| Autonomy will not start after manual | Mode `BLOCKED`: X off, then R1 off and on |
| Autonomy will not start at all | R1 on? `ros2 topic echo /hal/dead_man_switch`; is a controller running (`CONTROLLER`)? |
| Wrong button does something | YAML not loaded (see Settings), or a different pad layout: watch `/joy` while pressing |
| Throttle does nothing, mode is `MANUAL` | Pit limit or reverse on? `/control/teleop` shows both; `/control/speed_limiter` shows if the limiter holds it |
| Car slower than asked | Pit limit, then `speed_max` in `vesc.yaml`, then the VESC's Max ERPM |
| Reverse stutters | `reverse_speed` below 0.5 m/s |
| Toggles all reset | `joy_node` restarted (stack restart): sticky state lives there |
