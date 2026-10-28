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
"""Unit tests for the planner (no ROS graph needed)."""

from my_local_planner.planner import Params, State, plan

# A straight raceline along +x at 1 m/s, and a scan that sees nothing
STRAIGHT = [(0.1 * i, 0.0, 1.0) for i in range(50)]
EMPTY_SCAN = ([float("inf")] * 1080, -2.35, 0.00436)


def test_empty_track_keeps_the_raceline_or_plans_ahead():
    path = plan(State(), EMPTY_SCAN, STRAIGHT, Params())
    if path is not None:
        # a planned path starts at the car (base_link origin)
        assert path
        assert path[0][0] < 0.5


# TODO: once plan() is written, test its behaviour, for example an obstacle 2 m ahead on the
# raceline gives a path that passes it on one side.
