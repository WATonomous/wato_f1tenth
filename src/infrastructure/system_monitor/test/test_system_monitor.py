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
"""Unit tests for the system readers and black box, against a fake /proc + /sys tree (no ROS)."""

import csv
import math

import pytest

from system_monitor import readers
from system_monitor.blackbox import CLEAN_STOP, BlackBox


def write(root, path, text):
    f = root / path
    f.parent.mkdir(parents=True, exist_ok=True)
    f.write_text(text)


@pytest.fixture
def jetson(tmp_path):
    """A fake Orin: two cores, an INA3221 with VDD_IN, thermal zones, GPU load."""
    write(
        tmp_path,
        "proc/stat",
        "cpu  100 0 100 800 0 0 0 0\ncpu0 50 0 50 400 0\ncpu1 50 0 50 400 0\n",
    )
    write(tmp_path, "proc/meminfo", "MemTotal: 8000000 kB\nMemAvailable: 6000000 kB\n")
    write(tmp_path, "sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "1500000\n")
    write(tmp_path, "sys/devices/system/cpu/cpu1/cpufreq/scaling_cur_freq", "1700000\n")
    write(tmp_path, "sys/devices/platform/bus@0/17000000.gpu/load", "450\n")
    write(tmp_path, "sys/class/thermal/thermal_zone0/type", "cpu-thermal\n")
    write(tmp_path, "sys/class/thermal/thermal_zone0/temp", "52500\n")
    write(tmp_path, "sys/class/thermal/thermal_zone1/type", "gpu-thermal\n")
    write(
        tmp_path, "sys/class/thermal/thermal_zone1/temp", "-256000\n"
    )  # disabled sensor
    hw = "sys/class/hwmon/hwmon1"
    write(tmp_path, f"{hw}/name", "ina3221\n")
    write(tmp_path, f"{hw}/in1_label", "VDD_IN\n")
    write(tmp_path, f"{hw}/in1_input", "5000\n")  # mV
    write(tmp_path, f"{hw}/curr1_input", "2000\n")  # mA
    write(tmp_path, f"{hw}/curr1_crit_alarm", "0\n")
    write(tmp_path, f"{hw}/in2_label", "VDD_CPU_GPU_CV\n")
    write(tmp_path, f"{hw}/in2_input", "5000\n")
    write(tmp_path, f"{hw}/curr2_input", "800\n")
    write(tmp_path, f"{hw}/curr2_crit_alarm", "1\n")
    write(tmp_path, "sys/class/hwmon/hwmon0/name", "nvme\n")  # another chip, ignored
    return tmp_path


def test_cpu_percent_from_two_samples(jetson):
    before = readers.cpu_times(jetson)
    write(
        jetson,
        "proc/stat",
        "cpu  200 0 100 900 0 0 0 0\ncpu0 100 0 50 450 0\ncpu1 100 0 50 450 0\n",
    )
    after = readers.cpu_times(jetson)
    # all cores: 200 more ticks, 100 of them idle -> 50 % busy
    assert readers.cpu_percent(before, after) == pytest.approx([50.0, 50.0, 50.0])


def test_memory_frequency_gpu(jetson):
    assert readers.memory(jetson) == pytest.approx((25.0, 2000000 / 1024))
    assert readers.cpu_freq_mhz(jetson) == pytest.approx(1600.0)
    assert readers.gpu_percent(jetson) == pytest.approx(45.0)


def test_temperatures_skip_disabled_sensors(jetson):
    assert readers.temperatures(jetson) == [("cpu-thermal", pytest.approx(52.5))]


def test_power_rails_and_module_power(jetson):
    rails = readers.power_rails(jetson)
    assert [(r[0], r[4]) for r in rails] == [
        ("VDD_IN", False),
        ("VDD_CPU_GPU_CV", True),
    ]
    assert readers.module_power_w(rails) == pytest.approx(10.0)  # 5 V x 2 A


def test_missing_information_is_nan(tmp_path):
    assert math.isnan(readers.gpu_percent(tmp_path))
    assert math.isnan(readers.cpu_freq_mhz(tmp_path))
    assert readers.power_rails(tmp_path) == []
    assert math.isnan(readers.module_power_w([]))
    assert all(math.isnan(v) for v in readers.memory(tmp_path))


def test_total_power():
    # 30 W drive + (10 W Jetson + 4 W lidar) through 90 % efficient converters
    assert readers.total_power(30.0, 10.0, 4.0, 0.9) == pytest.approx(30.0 + 14.0 / 0.9)
    # no VESC data (sim): only the power-board branch
    assert readers.total_power(float("nan"), 10.0, 0.0, 1.0) == pytest.approx(10.0)


def test_blackbox_writes_flushes_and_prunes(tmp_path):
    clock = [1000.0]
    for i in range(3):  # three earlier runs
        (tmp_path / f"blackbox_2026010{i}_000000.csv").write_text("old\n")
    box = BlackBox(
        str(tmp_path), keep_files=2, flush_period_s=1.0, now=lambda: clock[0]
    )
    for _ in range(15):
        box.write({"pack_voltage_v": 11.5, "alarms": ""})
        clock[0] += 0.1
    # flushed at least once without close(): what a power cut would leave on disk
    with open(box.path) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) >= 10
    assert rows[0]["pack_voltage_v"] == "11.500"
    box.close()
    assert (
        len(list(tmp_path.glob("blackbox_*.csv"))) == 2
    )  # the newest old run + this one
    with open(box.path) as f:
        assert list(csv.DictReader(f))[-1]["alarms"] == CLEAN_STOP


def test_blackbox_never_overwrites_a_run_with_the_same_start_time(tmp_path):
    # after a power cut the Jetson can boot with the same wrong time as last time
    first = BlackBox(str(tmp_path), now=lambda: 1000.0)
    first.write({"pack_voltage_v": 11.0})
    first.flush()
    second = BlackBox(str(tmp_path), now=lambda: 1000.0)
    assert second.path != first.path
    with open(first.path) as f:
        assert list(csv.DictReader(f))[0]["pack_voltage_v"] == "11.000"
