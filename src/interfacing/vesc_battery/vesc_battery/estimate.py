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
"""Battery state from pack voltage and current. No ROS imports.

The estimate is voltage based. Coulomb counting from the VESC doesn't work on this car: the VESC
only measures its own input current, while the Jetson and sensors draw from the pack ahead of it.
"""

import math

# Resting (open-circuit) LiPo cell voltage -> state of charge, typical curve
LIPO_CURVE = [
    (3.27, 0.00),
    (3.61, 0.05),
    (3.69, 0.10),
    (3.71, 0.15),
    (3.73, 0.20),
    (3.75, 0.25),
    (3.77, 0.30),
    (3.79, 0.35),
    (3.80, 0.40),
    (3.82, 0.45),
    (3.84, 0.50),
    (3.85, 0.55),
    (3.87, 0.60),
    (3.91, 0.65),
    (3.95, 0.70),
    (3.98, 0.75),
    (4.02, 0.80),
    (4.08, 0.85),
    (4.11, 0.90),
    (4.15, 0.95),
    (4.20, 1.00),
]

OK = "ok"
LOW = "low"
CRITICAL = "critical"
OVERVOLTAGE = "overvoltage"


def cell_fraction(cell_v):
    """State of charge (0..1) of a resting LiPo cell, linear between curve points."""
    if cell_v <= LIPO_CURVE[0][0]:
        return 0.0
    if cell_v >= LIPO_CURVE[-1][0]:
        return 1.0
    for (v0, f0), (v1, f1) in zip(LIPO_CURVE, LIPO_CURVE[1:]):
        if cell_v <= v1:
            return f0 + (f1 - f0) * (cell_v - v0) / (v1 - v0)
    return 1.0


def resting_cell_voltage(pack_v, current_a, cells, internal_resistance_ohm):
    """Per-cell voltage with the load sag (I * R) added back, so driving doesn't read as empty."""
    return (pack_v + max(current_a, 0.0) * internal_resistance_ohm) / cells


def level(cell_v, warn_cell_v, critical_cell_v, max_cell_v=4.25):
    if cell_v > max_cell_v:
        return OVERVOLTAGE
    if cell_v < critical_cell_v:
        return CRITICAL
    if cell_v < warn_cell_v:
        return LOW
    return OK


class Smoother:
    """Exponential moving average with a time constant, robust to uneven sample spacing."""

    def __init__(self, time_constant_s):
        self.tau = time_constant_s
        self.value = None
        self.last_t = None

    def reset(self):
        self.value = None
        self.last_t = None

    def update(self, x, t):
        if self.value is None or self.last_t is None or t < self.last_t:
            self.value = x  # first sample, or time went backwards (bag loop): restart
        else:
            alpha = (
                1.0 - math.exp(-(t - self.last_t) / self.tau) if self.tau > 0 else 1.0
            )
            self.value += alpha * (x - self.value)
        self.last_t = t
        return self.value
