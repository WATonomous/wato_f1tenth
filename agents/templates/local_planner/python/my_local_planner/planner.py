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
"""The planner: everything you write lives in plan(). No ROS imports, so tests call it directly.

node.py handles topics, timing, stale inputs and the active flag.
"""

from dataclasses import dataclass


@dataclass
class Params:
    """Tunables, set from config/{car,sim}/my_local_planner.yaml. Add yours here; node.py
    declares every field as a parameter."""

    horizon_m: float = 3.0  # how far ahead to plan


@dataclass
class State:
    """The car, in the map frame."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0  # rad
    speed: float = 0.0  # m/s, from wheel odometry


def plan(state, scan, global_path, params):
    """Return the path the controller should follow now, or None to keep the global raceline.

    scan: (ranges, angle_min, angle_increment) in the laser frame.
    global_path: the raceline as a list of (x, y, target_speed), map frame, a closed loop.
    Returns a list of (x, y, target_speed) in base_link (x forward, y left), starting at the car.
    """
    # TODO: your planner. Until then the controller keeps following the global raceline.
    return None
