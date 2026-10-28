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
"""The control law: everything you write lives in compute(). No ROS imports, so tests call it directly.

node.py handles topics, the dead man switch, path choice and the watchdog.
"""

from dataclasses import dataclass


@dataclass
class Params:
    """Tunables, set from config/{car,sim}/my_controller.yaml. Add your gains here; node.py
    declares every field as a parameter."""

    max_speed: float = 1.0  # m/s
    max_steering_angle: float = 0.4  # rad
    wheelbase: float = 0.33  # m


@dataclass
class State:
    """The car, in the map frame."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0  # rad
    speed: float = 0.0  # m/s, from wheel odometry


@dataclass
class Command:
    steering_angle: float = 0.0  # rad, positive = left
    speed: float = 0.0  # m/s
    debug_point: tuple = (
        None  # optional (x, y) to show in Foxglove, in the path's frame
    )


def compute(state, path, path_is_local, params):
    """One control step.

    path is a list of (x, y, target_speed), either
      - the global raceline: map frame, a closed loop (path_is_local False), or
      - the local planner's path: base_link frame, open, starting at the car (path_is_local True).
    The node clamps the result to the limits.
    """
    # TODO: your control law. Until then the car stays stopped.
    return Command()
