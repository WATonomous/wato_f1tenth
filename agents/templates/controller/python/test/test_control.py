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
"""Unit tests for the control law (no ROS graph needed)."""

from my_controller.control import Params, State, compute

# A straight raceline along +x at 1 m/s
STRAIGHT = [(0.1 * i, 0.0, 1.0) for i in range(50)]


def test_compute_stays_within_the_limits():
    params = Params()
    cmd = compute(State(), STRAIGHT, False, params)
    assert abs(cmd.steering_angle) <= params.max_steering_angle
    assert 0.0 <= cmd.speed <= params.max_speed


# TODO: once compute() is written, test its behaviour, for example:
#   - on the line and facing along it: steering_angle is about 0
#   - offset to the right of the line (State(y=-0.5)): steering_angle > 0 (turns left)
