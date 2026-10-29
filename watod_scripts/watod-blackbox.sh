#!/usr/bin/env bash
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# watod blackbox [list | <file>]   read system_monitor's black box (logs/blackbox/ on the car)
#
#   watod blackbox          the last run that ended without a clean stop (a power cut or a crash):
#                           its last 20 seconds, one row a second, and the trend before the end
#   watod blackbox list     every run: start, length, and whether it stopped cleanly
#   watod blackbox <file>   the same report for a given file
#
# Runs on the host with plain python3; no containers needed.

set -euo pipefail

MONO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BLACKBOX_DIR="${BLACKBOX_DIR:-$MONO_DIR/logs/blackbox}"

python3 - "$BLACKBOX_DIR" "$@" <<'PY'
import csv, glob, math, os, sys

directory, args = sys.argv[1], sys.argv[2:]
CLEAN = "clean shutdown"

def load(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))

def num(row, key):
    try:
        return float(row.get(key) or "nan")
    except ValueError:
        return math.nan

def fmt(v, unit, width=6, decimals=1):
    return f"{'-':>{width}}" if math.isnan(v) else f"{v:{width}.{decimals}f}{unit}"

files = sorted(glob.glob(os.path.join(directory, "blackbox_*.csv")))
if not files:
    sys.exit(f"No black box files in {directory} (it is written on the car by system_monitor).")

if args[:1] == ["list"]:
    for path in files:
        rows = load(path)
        clean = bool(rows) and rows[-1].get("alarms") == CLEAN
        span = num(rows[-1], "unix_s") - num(rows[0], "unix_s") if rows else 0.0
        print(f"{os.path.basename(path)}  {span / 60:6.1f} min  {'clean stop' if clean else 'ENDED ABRUPTLY'}")
    sys.exit(0)

if args:
    path = args[0] if os.path.exists(args[0]) else os.path.join(directory, args[0])
else:
    abrupt = [p for p in files[:-1] if load(p) and load(p)[-1].get("alarms") != CLEAN]
    if not abrupt:
        sys.exit("No run ended abruptly; `watod blackbox list` shows all runs.")
    path = abrupt[-1]

rows = [r for r in load(path) if r.get("alarms") != CLEAN]
if not rows:
    sys.exit(f"{path} is empty.")
end = num(rows[-1], "unix_s")
print(f"{os.path.basename(path)}: last row at {rows[-1]['time']}")
print(f"{'time':>10} {'pack V':>8} {'drive A':>8} {'drive W':>8} {'Jetson W':>9} {'total W':>8} {'CPU %':>7} {'max C':>7}  alarms")
shown = set()
for r in rows:
    t = num(r, "unix_s")
    if end - t > 20 or int(t) in shown:
        continue
    shown.add(int(t))
    print(f"{r['time']:>10} {fmt(num(r, 'pack_voltage_v'), '', 8, 2)} {fmt(num(r, 'drive_current_a'), '', 8)} "
          f"{fmt(num(r, 'drive_power_w'), '', 8)} {fmt(num(r, 'compute_power_w'), '', 9)} "
          f"{fmt(num(r, 'total_power_w'), '', 8)} {fmt(num(r, 'cpu_percent'), '', 7)} "
          f"{fmt(num(r, 'max_temperature_c'), '', 7)}  {r.get('alarms', '')}")
last10 = [r for r in rows if end - num(r, "unix_s") <= 10]
volts = [num(r, "pack_voltage_v") for r in last10 if not math.isnan(num(r, "pack_voltage_v"))]
watts = [num(r, "total_power_w") for r in last10 if not math.isnan(num(r, "total_power_w"))]
print()
if volts:
    print(f"last 10 s: pack {min(volts):.2f}-{max(volts):.2f} V (last {volts[-1]:.2f} V)", end="")
if watts:
    print(f", total power up to {max(watts):.1f} W", end="")
print()
print("Pack voltage steady to the end while the car went down: look at the battery -> power board")
print("connection and the board. Voltage sagging first: the battery itself.")
PY
