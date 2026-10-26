# Deploy to the car

How code gets onto the Jetson and how the car is brought up. Agents prepare changes and
write commands; **a human runs everything on the car** (`AGENTS.md` rules 1 and 2).
Sources: `docs/mppi/` (from `mppi_test`) and the old `jetson_pf` / `jetson_up` launch files.

## Status on `big_infra_update`

With `PLATFORM=car`, `watod` adds `modules/docker-compose.car.yaml`: `interfacing` gets the host's
`/dev` (privileged, plus `/run/udev` and the device cgroup rules below), so USB and Bluetooth
devices plugged in after start-up appear without restarting the container. `world_modeling`
and `action` get `runtime: nvidia`. Until the stack has been driven on `big_infra_update`, keep
the `orin` / `mppi_test` checklist below as the fallback.

## The hardware

- Jetson Orin Nano Super or Orin NX (both `sm_87`), on JetPack 6 (L4T r36.4).
- Hokuyo lidar over Ethernet at `192.168.0.10:10940`, frame `laser`.
- VESC motor controller at `/dev/sensors/vesc` (a udev symlink).
- PS4 controller over Bluetooth, read by `joy_node`.
- Remote access over Tailscale, with a desktop over noVNC.

## Bring-up checklist (old layout: `orin` / `mppi_test`)

1. **Battery.** Plug it in the right way round: red to red, black to black.
2. **Access.** Start Tailscale on the laptop, and open the car's noVNC desktop in a browser.
3. **Controller.** Put the PS4 pad in pairing mode. In the car's Bluetooth settings, remove the old
   "Wireless Controller", search again, and pair it.
4. **Display.** In a noVNC terminal: `xhost +local:root` (needed for RViz from the container).
5. **Power mode** (optional, for full performance):
   `sudo nvpmodel -m 2` (MAXN_SUPER on the Orin Nano Super; mode numbers differ per module,
   so check `sudo nvpmodel -q --verbose` on an NX), then `sudo jetson_clocks`.
6. **Containers.** In the repo on the car: `./watod up` (develop mode).
7. **Build.** `docker exec -it watod_<user>-robot_dev-1 bash`, then
   `source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash`.
8. **Launch.**
   - Drivers, teleop, mux, e-brake and EKF only: `ros2 launch bringup_robot jetson_up.launch.py`
   - Full stack with localization: set `OPENBLAS_NUM_THREADS=1` and `OMP_NUM_THREADS=1` first,
     then `ros2 launch bringup_robot jetson_pf.launch.py` (this is what `start_pf.sh` does).
9. **Core pinning** (optional, when running MPPI):
   `sudo taskset -cp 0-2 $(pgrep -f particle_filter)`, `3-5` for `mppi_node`, `5` for
   `opponent_predictor`.
10. **Check.** `ros2 topic hz /scan`, `/odom`, `/pf/pose/odom`; drive with the controller while
    holding the dead-man button before enabling autonomy.

## Container access to devices

`interfacing` (via `docker-compose.car.yaml`) and the old `robot_dev` service run `privileged`,
mount `/dev`, and allow these device cgroups: `13:*` (input / joystick), `188:0` (USB serial), `166:*` (ACM serial, VESC) and
`10:229` (`/dev/uhid`, Bluetooth HID). If a driver can't open its device, check these first.

## Known traps

- Old layout only: `jetson_pf.launch.py` runs the map server with `use_sim_time: True`
  (fixed in `world_modeling.launch.yaml`).
- `teleop_utils`' `joypad_config.yaml` is keyed `joypad_node:` but the node is `gamepad_node`,
  so its settings are ignored on `orin` (fixed with `/**:` on `mppi_test`).
- `jetson_up.launch.py` defines a `throttle_interpolator` node from `f1tenth_stack`, which is not
  in the repo. It is not added to the launch description, so leave it unused.
- Old layout only: compose's `deploy` command points at `robot.launch.py`, which does not exist
  on the car branches, so use develop mode and launch by hand there.
- Remote Foxglove or RViz subscribers to heavy debug topics can slow the control loop
  (`rules.md` rule 10).

## On `big_infra_update`

```bash
# watod-config.local.sh on the Jetson
export ACTIVE_MODULES="interfacing world_modeling action"   # interfacing alone = old jetson_up
export PLATFORM="car"
./watod build-base      # once: CPU + Jetson CUDA/TensorRT bases (step 5 replaces this with a pull)
./watod build
./watod up              # deploy mode: no manual colcon build
```

| Old launch | New equivalent |
| --- | --- |
| `jetson_up.launch.py` | `ACTIVE_MODULES="interfacing"`, `PLATFORM=car` |
| `jetson_pf.launch.py` | `interfacing world_modeling` (+ `action` for pure pursuit), `PLATFORM=car` |
| `jetson_mapping.launch.py` | `interfacing world_modeling`, `PLATFORM=car`, `MAPPING=true` |
| RViz in jetson_pf | `world_modeling.launch.yaml rviz:=true` |
| e-brake, EKF | `interfacing.launch.yaml ebrake:=true`, `world_modeling.launch.yaml ekf:=true` (both off) |

Power mode, clocks and core pinning stay manual (checklist steps 5 and 9). After step 5 the car
pulls prebuilt arm64 images from GHCR, pinned to a dated `car-YYYY-MM-DD` tag on race days.
