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
10. **Check.** `ros2 topic hz /scan`, `/odom`, `/pf/pose/odom`. Gamepad buttons toggle:
    **X** turns manual control on and off (it beats autonomy); **R1**, the dead man switch, turns
    autonomy on and off. After manual control, toggle R1 off and on to hand back to autonomy.
    Try both with the wheels off the ground first.

## Container access to devices

`interfacing` (via `docker-compose.car.yaml`) and the old `robot_dev` service run `privileged`,
mount `/dev`, and allow these device cgroups: `13:*` (input / joystick), `188:0` (USB serial), `166:*` (ACM serial, VESC) and
`10:229` (`/dev/uhid`, Bluetooth HID). If a driver can't open its device, check these first.

## One-time Jetson host setup

A human runs these once per Jetson (they need sudo); re-run them after a re-flash.

```bash
# 1. Keep the kernel's webcam driver off the RealSense. librealsense (RSUSB backend, needed for
#    the IMU) and uvcvideo otherwise fight over the camera about once a second; under color +
#    depth load that stalls the IMU ("Motion Module force pause") and the VESC on the same hub.
echo "blacklist uvcvideo" | sudo tee /etc/modprobe.d/blacklist-uvcvideo.conf
sudo modprobe -r uvcvideo

# 2. Pin CPU and GPU clocks at boot. Without jetson_clocks the GPU idles at 306 MHz and the
#    particle filter (GPU ray casting) drops to ~40 Hz; with it, ~50 Hz.
sudo tee /etc/systemd/system/jetson-clocks.service > /dev/null <<'EOF'
[Unit]
Description=jetson_clocks: pin CPU/GPU/EMC clocks for the F1TENTH stack
After=nvpmodel.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl enable --now jetson-clocks.service

# 3. Docker DNS (see Known traps): "dns": ["8.8.8.8", "1.1.1.1"] in /etc/docker/daemon.json
```

Check: `./watod contract` passes with `/hal/localization/odom` at 40 Hz or more (goal 50).

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
- `./watod build` fails with "Temporary failure resolving" on some Wi-Fi networks: build steps
  get the host's IPv6 DNS servers first and can't reach them. The Jetson's
  `/etc/docker/daemon.json` sets `"dns": ["8.8.8.8", "1.1.1.1"]` (next to the nvidia runtime);
  keep it when re-flashing.
- A full `./watod build` on the 8 GB Jetson builds several images at once and can run out of
  memory (the build is killed, or the Jetson freezes and needs a power cycle). When more than
  one module needs rebuilding, build them one at a time:
  `for m in infrastructure interfacing world_modeling action; do ./watod -m $m build || break; done`.
  Keep the car plugged in while building (`build.md`, "On the car").

## On `big_infra_update`

```bash
# watod-config.local.sh on the Jetson. PLATFORM=car and ACTIVE_MODULES="interfacing
# world_modeling action" are detected; set them only to override (interfacing alone = old jetson_up)
export REALSENSE="true"         # RealSense IMU on /hal/imu (IMU=auto picks it on the car)
export REALSENSE_IMAGES="true"  # also color + depth 640x480 @ 30 Hz, for recording
export CONTROLLER="none"        # bench testing: no pure pursuit (default pure_pursuit)
./watod build-base      # once: CPU + Jetson CUDA/TensorRT bases (step 5 replaces this with a pull)
./watod build           # or one module at a time (see Known traps)
./watod up              # deploy mode: no manual colcon build
./watod contract        # every /hal topic at its rate; the particle filter must hold 40 Hz
```

| Old launch | New equivalent |
| --- | --- |
| `jetson_up.launch.py` | `ACTIVE_MODULES="interfacing"`, `PLATFORM=car` |
| `jetson_pf.launch.py` | `interfacing world_modeling` (+ `action` for pure pursuit), `PLATFORM=car` |
| `jetson_mapping.launch.py` | `interfacing world_modeling`, `PLATFORM=car`, `MAPPING=true` |
| RViz in jetson_pf | `world_modeling.launch.yaml rviz:=true` |
| e-brake, EKF | `interfacing.launch.yaml ebrake:=true`, `world_modeling.launch.yaml ekf:=true` (both off) |

Agents run these over SSH from the developer's machine, never logged into the car: see
`remote-car.md`.

Power mode and core pinning stay manual (checklist steps 5 and 9); clocks come from the
`jetson-clocks` service (One-time Jetson host setup). After step 5 the car
pulls prebuilt arm64 images from GHCR, pinned to a dated `car-YYYY-MM-DD` tag on race days.

## Recording on the car

```bash
./watod bag record   # car preset by default (BAG_PRESET); Ctrl-C to stop; saved under ./bags
```

The `car` preset (`infrastructure_bringup/config/bag_presets/car.txt`) records the `/hal`
contract (including `/hal/battery`), TF, the map and plan, VESC state, the gamepad and, with `REALSENSE_IMAGES=true`, color
(JPEG) and depth (zstd) at about 30 Hz. It leaves out `/pf/viz/*`: the particle filter does extra
ray casting and sampling for every subscriber to those, which costs it its 40 Hz floor. They are
off unless `PF_VIZ=true` (debugging localization); avoid `record -a` on the car with them on.
The recorder container runs on CPUs 4 and 5 (`BAG_CPUSET`). The Orin Nano has no
hardware video encoder, so image compression runs on the CPU in the RealSense driver.

## Battery

`vesc_battery` publishes the drive pack on `/hal/battery` (`sensor_msgs/BatteryState`): voltage,
an estimated charge (resting-LiPo curve with the load sag added back), and health (`DEAD` =
critical). It logs a warning below 3.7 V per cell and an error below 3.6 V per cell; the Jetson runs
off the same pack, so treat critical as "stop now". The VESC only measures its own current, so the
estimate is voltage based. To watch it, import `infrastructure_bringup/config/foxglove/battery.json`
in Foxglove (Layouts > Import from file).
