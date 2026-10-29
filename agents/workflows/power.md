# Power: what feeds what, and finding the cause of a cut-out

The car has lost power suddenly more than once, including during a build with the motor idle. This
file covers how the car is powered, what the stack measures, and how to read the black box after a
cut-out.

## Wiring

```text
LiPo pack ──┬── VESC ──────────────── drive motor, steering servo
            └── F1TENTH power board v2024.1 (battery input J9, a 4-pin XH-style header)
                  ├── 19 V barrel jack ── Jetson Orin Nano     (max 4 A)
                  ├── 12 V ────────────── Hokuyo lidar, terminal blocks (2 A total)
                  └── 5 V ─────────────── terminal blocks      (1 A total)
```

- **The board takes a 3S pack only: 9 V minimum, 11.1 V nominal, 12.6 V maximum** (board manual,
  Table 1). From the schematic, its 19 V converter (LM5176) turns on at about 8.5 V. A 2S pack (8.4 V
  full) is below the board's input range. The VESC read 11.4–12.8 V on 2026-10-27, which is a 3S pack.
- The battery input is a small header behind a 5 A Schottky diode. The Jetson at full CPU plus the
  lidar draws about 2.5–3 A from an 11 V pack. That leaves little margin for a worn connector or thin
  wires.

## What the stack measures

| Topic | From | What |
| --- | --- | --- |
| `/hal/battery` | `vesc_battery` (interfacing) | pack voltage, estimated charge, health |
| `/system/power` (10 Hz, car) | `system_monitor` (infrastructure) | pack voltage and drive current (VESC), Jetson module power (INA3221 `VDD_IN`), estimated total draw |
| `/system/status` (1 Hz) | `system_monitor` | CPU, GPU, RAM, disk, temperatures, the Jetson's power rails, alarms |

The total is an estimate: drive power + (Jetson module power + `other_loads_w`) /
`converter_efficiency`, set in `infrastructure_bringup/config/car/system_monitor.yaml`. The lidar,
the carrier board and USB devices aren't measured. Check those two values with an inline power meter
before relying on the total for choosing a battery.

## Black box

On the car, `system_monitor` writes `logs/blackbox/blackbox_<start>.csv` in the repo: pack voltage,
drive current, Jetson power, total power, CPU, temperature and alarms, 10 times a second. It flushes
to disk every second, so a cut-out loses at most the last second. The last 30 runs are kept. A clean
stop writes a final `clean shutdown` row. `./watod down` stops containers with SIGINT (compose
`stop_signal`), which gives every node a clean shutdown; a run without that row ended in a power cut,
a crash, or a `docker kill`.

```bash
./watod blackbox list     # every run: start, length, clean stop or ENDED ABRUPTLY
./watod blackbox          # the last abrupt run: its final 20 s, and the trend before the end
```

## After a cut-out

1. Power the car back on, then run `./watod blackbox` on the car (it needs no containers).
2. Read the end of the run:
   - **Pack voltage steady to the last row:** the battery was fine. Look between the battery and
     the Jetson: the split, the board's input header and wires, the board, the barrel jack.
   - **Pack voltage sagging first:** the battery. It may be weak, too small for the load, or nearly
     empty. Compare with the total power at the time.
   - **Jetson power or CPU jumping just before the end** (a build, `jetson_clocks`): the load at
     that moment is what to reproduce on the bench.
3. Reproduce on the bench with the same load, watching `/system/power` in Foxglove.

## Builds on the car

Building the images keeps the Jetson at full CPU for minutes, and a build is what took the car down
on 2026-10-27. Build with the car plugged into power (`build.md`).
