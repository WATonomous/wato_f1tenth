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
"""Unit tests for the battery estimate (no ROS graph needed)."""

import pytest

from vesc_battery import estimate


@pytest.mark.parametrize(
    "cell_v,expected", [(3.0, 0.0), (3.27, 0.0), (3.84, 0.5), (4.2, 1.0), (4.3, 1.0)]
)
def test_cell_fraction_endpoints_and_midpoint(cell_v, expected):
    assert estimate.cell_fraction(cell_v) == pytest.approx(expected)


def test_cell_fraction_is_monotonic():
    values = [estimate.cell_fraction(3.2 + i * 0.005) for i in range(220)]
    assert values == sorted(values)


def test_cell_fraction_interpolates():
    # halfway between (3.80, 0.40) and (3.82, 0.45)
    assert estimate.cell_fraction(3.81) == pytest.approx(0.425)


def test_recorded_car_voltage_is_mid_charge():
    # 11.4 V at rest on the 3S pack (loadtest_p1 bag), about 3.8 V per cell
    cell = estimate.resting_cell_voltage(11.4, 0.0, 3, 0.03)
    assert cell == pytest.approx(3.8)
    assert 0.35 < estimate.cell_fraction(cell) < 0.45


def test_load_sag_is_compensated_and_regen_ignored():
    assert estimate.resting_cell_voltage(11.1, 10.0, 3, 0.03) == pytest.approx(
        (11.1 + 0.3) / 3
    )
    assert estimate.resting_cell_voltage(11.4, -5.0, 3, 0.03) == pytest.approx(3.8)


@pytest.mark.parametrize(
    "cell_v,expected",
    [
        (3.9, estimate.OK),
        (3.65, estimate.LOW),
        (3.55, estimate.CRITICAL),
        (4.3, estimate.OVERVOLTAGE),
    ],
)
def test_levels(cell_v, expected):
    assert estimate.level(cell_v, warn_cell_v=3.7, critical_cell_v=3.6) == expected


def test_smoother_converges_and_restarts():
    s = estimate.Smoother(1.0)
    assert s.update(10.0, 0.0) == 10.0
    for i in range(1, 101):
        s.update(12.0, i * 0.1)
    assert s.value == pytest.approx(12.0, abs=1e-3)
    assert s.update(5.0, 0.0) == 5.0  # time went backwards: restart
    s.reset()
    assert s.value is None
    assert s.update(7.0, 1.0) == 7.0
