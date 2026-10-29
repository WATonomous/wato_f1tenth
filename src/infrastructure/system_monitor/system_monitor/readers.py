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
"""Read computer health from /proc and /sys. No ROS, so tests run them against a fake tree.

Every reader takes the filesystem root (default "/") and returns NaN or an empty list when the
machine doesn't have that information (no GPU load file, no power monitor, ...).
"""

import glob
import math
import os

NAN = float("nan")

# GPU load files, 0-1000 (per mille): Jetson Orin (JetPack 6) and older Jetsons
GPU_LOAD_FILES = (
    "sys/devices/platform/bus@0/17000000.gpu/load",
    "sys/devices/platform/17000000.ga10b/load",
    "sys/devices/gpu.0/load",
)


def _read(root, path):
    try:
        with open(os.path.join(root, path)) as f:
            return f.read().strip()
    except OSError:
        return None


def _read_float(root, path):
    text = _read(root, path)
    try:
        return float(text)
    except (TypeError, ValueError):
        return NAN


def cpu_times(root="/"):
    """[(total, idle)] for all cores together, then each core, from /proc/stat."""
    out = []
    for line in (_read(root, "proc/stat") or "").splitlines():
        if not line.startswith("cpu"):
            continue
        values = [int(v) for v in line.split()[1:]]
        idle = values[3] + (values[4] if len(values) > 4 else 0)  # idle + iowait
        out.append((sum(values), idle))
    return out


def cpu_percent(before, after):
    """Busy percentage per entry between two cpu_times() samples."""
    out = []
    for (t0, i0), (t1, i1) in zip(before, after):
        total = t1 - t0
        out.append(100.0 * (1.0 - (i1 - i0) / total) if total > 0 else 0.0)
    return out


def cpu_freq_mhz(root="/"):
    files = glob.glob(
        os.path.join(root, "sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq")
    )
    freqs = [_read_float(root, os.path.relpath(f, root)) for f in files]
    freqs = [f for f in freqs if not math.isnan(f)]
    return sum(freqs) / len(freqs) / 1000.0 if freqs else NAN  # kHz -> MHz


def memory(root="/"):
    """(used percent, used MB) from MemTotal - MemAvailable."""
    info = {}
    for line in (_read(root, "proc/meminfo") or "").splitlines():
        key, _, rest = line.partition(":")
        if rest.split():
            info[key] = int(rest.split()[0])  # kB
    total, available = info.get("MemTotal"), info.get("MemAvailable")
    if not total or available is None:
        return NAN, NAN
    used = total - available
    return 100.0 * used / total, used / 1024.0


def disk_used_percent(path="/"):
    try:
        st = os.statvfs(path)
    except OSError:
        return NAN
    total = st.f_blocks * st.f_frsize
    return 100.0 * (total - st.f_bavail * st.f_frsize) / total if total else NAN


def gpu_percent(root="/"):
    for path in GPU_LOAD_FILES:
        value = _read_float(root, path)
        if not math.isnan(value):
            return value / 10.0
    return NAN


def temperatures(root="/"):
    """[(name, celsius)] from /sys/class/thermal; drops unplausible readings (disabled sensors)."""
    out = []
    for zone in sorted(
        glob.glob(os.path.join(root, "sys/class/thermal/thermal_zone*"))
    ):
        rel = os.path.relpath(zone, root)
        name = _read(root, f"{rel}/type")
        celsius = _read_float(root, f"{rel}/temp") / 1000.0
        if name and -40.0 < celsius < 150.0:
            out.append((name, celsius))
    return out


def power_rails(root="/"):
    """[(name, volts, amps, watts, alarm)] for each INA3221 channel (the Jetson's power monitor).

    alarm is True when the chip's critical or warning current alarm for that channel is set.
    """
    out = []
    for hwmon in sorted(glob.glob(os.path.join(root, "sys/class/hwmon/hwmon*"))):
        rel = os.path.relpath(hwmon, root)
        if _read(root, f"{rel}/name") != "ina3221":
            continue
        for ch in (1, 2, 3, 4):
            label = _read(root, f"{rel}/in{ch}_label")
            if not label or label.upper() == "NC":
                continue
            volts = _read_float(root, f"{rel}/in{ch}_input") / 1000.0
            amps = _read_float(root, f"{rel}/curr{ch}_input") / 1000.0
            alarm = any(
                _read(root, f"{rel}/curr{ch}_{kind}_alarm") == "1"
                for kind in ("crit", "max")
            )
            out.append((label, volts, amps, volts * amps, alarm))
    return out


def module_power_w(rails):
    """Jetson module input power: the VDD_IN rail, NaN without one."""
    for name, _, _, watts, _ in rails:
        if name.upper() in ("VDD_IN", "VIN_SYS_5V0", "POM_5V_IN"):
            return watts
    return NAN


def total_power(drive_w, compute_w, other_loads_w, converter_efficiency):
    """Estimated power drawn from the pack: drive branch + power-board branch through its converters."""
    board = (0.0 if math.isnan(compute_w) else compute_w) + other_loads_w
    drive = 0.0 if math.isnan(drive_w) else drive_w
    return drive + board / converter_efficiency
