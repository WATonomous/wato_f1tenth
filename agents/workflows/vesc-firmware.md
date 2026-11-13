# VESC firmware settings: VESC Tool, backups, motor tuning

How to read, back up, change and test the VESC's motor settings from this machine over SSH,
with the headless scripts in `agents/scripts/vesc/`. These settings live on the VESC, not in the
repo: ROS only sends speed and servo commands. Read `workflows/remote-car.md` first.

## The hardware

- VESC 6 MkV (`60_MK5`), **firmware 6.00**. VESC Tool must be the same version (`release_6_00`),
  or it opens in limited mode and refuses to write. Upgrading the firmware is a separate job that
  can reset the motor config: do not do it unless a human asks.
- Traxxas Velineon 3500 sensorless motor on stock Fiesta ST Rally gearing, 3S 5000 mAh pack.
  FOC, sensorless, observer `MXLEMMING_LAMBDA_COMP`. No motor temperature sensor (reads -75 C).
- `speed_to_erpm_gain` 4620 ERPM per m/s (`interfacing_bringup/config/car/vesc.yaml`), so
  10 m/s = 46200 ERPM. The stock gearing tops out around 14-16 m/s at 95% duty.

## Safety, every time

- These scripts touch `/dev/ttyACM0`, so AGENTS.md rule 2 applies: an agent runs one only after
  the human's explicit go for that run. Without it, write the command down for the human.
- **Ask before every step that moves the motor**, and say what will happen, for how long and how
  fast. Wait for the human's "go". Off-ground tests need the car on a stand with the wheels clear;
  ground tests need clear floor (3-5 m) and a person ready to catch the car.
- **Back up before any write** (`save_configs`), and keep `set_mcconf`'s `expect` checks.
  Change one thing per test, so a bad result points at one setting.
- Only one program can hold the VESC's serial port: the stack must be down (`./watod down`, ask
  first). Use `/dev/ttyACM0`; VESC Tool rejects the `/dev/sensors/vesc` symlink.
- Every test script feeds commands at 20-50 Hz and aborts on any fault, an RPM limit or a hard
  timeout; if the script dies, the VESC's 1 s command timeout stops the motor.
- Never ask for the car's password. `sudo` (apt installs) goes to the human.

## Build VESC Tool on the car (once)

Outside Docker, on the Jetson (Ubuntu 22.04, apt Qt 5.15). The human runs the apt line:

```bash
sudo apt install build-essential git qtbase5-dev qtbase5-private-dev qtdeclarative5-dev \
  qtquickcontrols2-5-dev libqt5svg5-dev libqt5serialport5-dev libqt5gamepad5-dev \
  qtconnectivity5-dev qtpositioning5-dev qml-module-qtquick-controls2 \
  qml-module-qtquick-layouts qml-module-qtquick-dialogs qml-module-qt-labs-settings
```

Then over SSH, detached with a log (keep the stack down: the build needs the RAM):

```bash
git clone -b release_6_00 https://github.com/vedderb/vesc_tool.git ~/vesc_tool && cd ~/vesc_tool
# the checkout has no bundled firmware; make loops on the missing res_fw.qrc, and an empty qrc breaks rcc
mkdir -p res/firmwares && echo "No firmware binaries bundled in this local build." > res/firmwares/README
printf '<RCC>\n    <qresource prefix="/res/firmwares">\n        <file>README</file>\n    </qresource>\n</RCC>\n' > res/firmwares/res_fw.qrc
qmake -config release "CONFIG += release_lin build_original"
nohup bash -c 'make -j4 > ~/vesc_tool_build.log 2>&1; echo "EXIT $?" >> ~/vesc_tool_build.log' >/dev/null 2>&1 &
# result: ~/vesc_tool/build/lin/vesc_tool_6.00 (GUI: run it from the noVNC desktop)
```

VESC Tool may offer a `pkexec` prompt to add the user to `dialout`; it is not needed.

## Headless scripts (`agents/scripts/vesc/`)

VESC Tool runs a QML file with no window: `run_vesc_script.sh <name>` wraps
`QT_QPA_PLATFORM=offscreen QT_QUICK_BACKEND=software vesc_tool_6.00 --loadQml <name>.qml`, refuses
to run while the interfacing container is up, and logs to `~/vesc_configs/tests/<time>_<name>.log`.
Run it from the car's checkout of this repo.

| Script | Moves the motor | Does |
| --- | --- | --- |
| `save_configs` | no | reads motor + app config to `~/vesc_configs/latest/`, copies to `backups/<time>/` |
| `set_mcconf` | no | applies `~/vesc_configs/changes.json` (format: `changes.example.json`) |
| `measure_motor` | **yes, off ground** | measures R, L (twice) and flux linkage; writes nothing |
| `offground_test` | **yes, off ground** | 3000 then 6000 ERPM, coast, small steering sweep |
| `highspeed_offground` | **yes, off ground** | 10k-46.2k ERPM steps (about 2-10 m/s), brake to 5k; aborts > 70 C or < 10.5 V |
| `ground_test` | **yes, on the floor** | two launches: 2500 and 4500 ERPM, 50 Hz log |
| `current_sweep` | **yes, on the floor** | fixed-current launches at 8, 15, 25 A (no speed PID) |
| `brake_test` | **yes, on the floor** | 4000 ERPM, step down to 1500, coast: does the PID brake? |
| `imu_check` | no | which IMU the firmware set up, the app config's IMU settings, and 20 IMU samples (on this VESC: BMI160 reported, but all zeros, 2026-11-12) |

`set_mcconf` reads the config, checks every `expect` (aborts and writes nothing on a mismatch or an
unknown name), snapshots `history/<time>_before_<label>.xml`, writes, reads back, and snapshots
`_after_`. Start with `"dry_run": true`. Parameter names are the VESC config names (open a saved
`mcconf.xml` to find them). The analysis scripts (`analyze_ground.py`, `analyze_sweep.py`,
`analyze_brake.py`) summarize a test log: copy the log back with `scp` and run them here.

Restore a backup: write a `changes.json` that sets the changed values back (diff the before/after
XML), or load the XML in the VESC Tool GUI (File > Load Motor Configuration XML, then Write).

## Tuning loop

1. `save_configs`, then copy the backup to this machine (`~/wato/vesc_configs/`).
2. Pick one change and say why. Dry run it, then write it after the human's "go".
3. Run the matching test after another "go"; analyze the log; compare against the last good run.
4. Keep it or revert it (a `changes.json` with `expect`/`value` swapped). Run twice when a result
   could be luck: launches failed about one time in three before the fix.
5. Report the settings table (original vs now) after every change.

## What the 2026-11 tuning found (tests 1-17)

The original tune was done with the wheels off the ground, which hides launch problems.

- **The launch stutter came from the open-loop timing.** `foc_sl_openloop_hyst` and
  `foc_sl_openloop_time` were 0.01 s (from the F1TENTH guide); 0.1 s each fixed it.
- **Lower handover is better on this motor.** Raising `foc_openloop_rpm_low` to 1.0 (handover at
  1200 ERPM) made launches worse twice: the loaded car cannot follow the open-loop ramp that fast.
- `foc_sl_openloop_time_lock` 0.05 s lines the rotor up first and removed the twitch at standstill.
- The Kd of 0.001 amplified the noisy low-speed speed estimate; Kp/Ki/Kd now 0.007/0.002/0.0002.
- **`s_pid_allow_braking` was off**, so the car only coasted when asked to slow down.
- **The VESC speed ramp is symmetric**: it limits braking as hard as speeding up. Keep it above
  the ROS limiter (`ackermann_to_vesc` 3.3 / 4.0 m/s^2) and let ROS shape the motion.
- Two limits on top speed: ROS `speed_max` in `vesc.yaml` and the VESC's `l_max_erpm`. Raise both
  together; the VESC limit should sit above the ROS one.
- At 10 m/s off the ground: no faults, MOSFETs about 31 C.

| Setting | Original | Now |
| --- | --- | --- |
| `foc_sl_openloop_hyst`, `foc_sl_openloop_time` | 0.01 s | 0.1 s |
| `foc_sl_openloop_time_lock` | 0 | 0.05 s |
| `foc_start_curr_dec` / `_rpm` | 1.0 / 2500 | 0.5 / 5000 |
| `s_pid_kp` / `ki` / `kd` | 0.015 / 0.00005 / 0.001 | 0.007 / 0.002 / 0.0002 |
| `s_pid_allow_braking` | off | on |
| `s_pid_ramp_erpms_s` | 15000 | 25000 (5.4 m/s^2) |
| `l_max_erpm` | 30000 | 50000 (10.8 m/s) |
| `l_in_current_min` (regen into the pack) | -20 A | -10 A (2C) |
| `l_battery_cut_start` / `_end` | 10.0 / 8.0 V | 10.2 / 9.6 V |

Original configs: `~/vesc_configs/2026-11-08_original/` on the car and in `~/wato/vesc_configs/`
on the maintainer's laptop. Every later write has before/after snapshots in `~/vesc_configs/history/`.
