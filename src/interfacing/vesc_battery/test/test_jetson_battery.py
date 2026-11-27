# Copyright (c) 2026-present WATonomous. All rights reserved.
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
"""Unit tests for the INA226 4S Jetson battery driver and estimator (no ROS graph or I2C hardware)."""

import math
import struct

import pytest

from vesc_battery import estimate, ina226
from vesc_battery.jetson_battery_node import JetsonBatteryMonitor


@pytest.mark.parametrize(
    "raw_word,scale,expected_v",
    [
        (0, 1.0, 0.0),
        (11840, 1.0, 14.8),  # nominal 4S (3.70 V/cell)
        (13440, 1.0, 16.8),  # full 4S (4.20 V/cell)
        (13440, 1.0012, 16.8 * 1.0012),  # 1 kOhm safety resistor calibration
        (0x8000 | 11840, 1.0, 14.8),  # reserved bit 15 masked defensively
    ],
)
def test_ina226_decode_bus_voltage(raw_word, scale, expected_v):
    raw_bytes = struct.pack(">H", raw_word)
    assert ina226.decode_bus_voltage(raw_bytes, scale) == pytest.approx(expected_v)


def test_ina226_decode_rejects_short_read():
    with pytest.raises(ValueError):
        ina226.decode_bus_voltage(b"\x00")


def test_ina226_chip_id_validation():
    assert ina226.is_valid_chip_id(0x5449, 0x2260)
    assert ina226.is_valid_chip_id(0x5449, 0x2261)  # revision nibble ignored
    assert not ina226.is_valid_chip_id(0x0000, 0x2260)
    assert not ina226.is_valid_chip_id(0x5449, 0x2270)


def test_jetson_4s_monitor_full_nominal_low_critical_and_unplugged():
    mon = JetsonBatteryMonitor(
        cells=4,
        capacity_ah=5.0,
        internal_resistance_ohm=0.04,
        estimated_load_a=0.0,
        smoothing_s=0.0,
        warn_cell_v=3.65,
        critical_cell_v=3.50,
    )

    # Full 4S pack (16.8 V -> 4.20 V/cell -> 100%, OK)
    snap = mon.update(16.8, 1.0)
    assert snap["present"] is True
    assert snap["voltage"] == pytest.approx(16.8)
    assert snap["cell_v"] == pytest.approx(4.2)
    assert snap["percentage"] == pytest.approx(1.0)
    assert snap["charge"] == pytest.approx(5.0)
    assert snap["level"] == estimate.OK

    # Low 4S pack (14.4 V -> 3.60 V/cell -> LOW)
    snap = mon.update(14.4, 2.0)
    assert snap["present"] is True
    assert snap["level"] == estimate.LOW

    # Critical 4S pack (13.8 V -> 3.45 V/cell -> CRITICAL)
    snap = mon.update(13.8, 3.0)
    assert snap["present"] is True
    assert snap["level"] == estimate.CRITICAL

    # Overvoltage (> 4.25 V/cell -> 17.2 V)
    snap = mon.update(17.2, 4.0)
    assert snap["present"] is True
    assert snap["level"] == estimate.OVERVOLTAGE

    # Unplugged balance plug (0 V or NaN) -> present=False and history reset
    snap = mon.update(0.0, 5.0)
    assert snap["present"] is False
    assert snap["voltage"] == pytest.approx(0.0)
    assert math.isnan(snap["percentage"])

    snap = mon.update(float("nan"), 6.0)
    assert snap["present"] is False
    assert math.isnan(snap["voltage"])


def test_jetson_4s_monitor_load_sag_compensation_and_reconnect():
    mon = JetsonBatteryMonitor(
        cells=4,
        capacity_ah=5.0,
        internal_resistance_ohm=0.05,
        estimated_load_a=2.0,  # +0.10 V sag comp -> +0.025 V/cell
        smoothing_s=3.0,
        warn_cell_v=3.65,
        critical_cell_v=3.50,
    )
    # 15.26 V + 2.0 * 0.05 = 15.36 V -> 3.84 V/cell (50% on LIPO_CURVE)
    snap = mon.update(15.26, 1.0)
    assert snap["present"] is True
    assert snap["percentage"] == pytest.approx(0.5)

    # Disconnect resets smoother so reconnect jumps immediately to the new pack voltage
    assert mon.update(float("nan"), 2.0)["present"] is False
    snap = mon.update(16.8, 3.0)
    assert snap["voltage"] == pytest.approx(16.8)
    assert snap["percentage"] == pytest.approx(1.0)
