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
"""The status panel's one word, from the input in control and the dead man. No ROS, for tests."""

MUX_STATES = {
    "joystick": "MANUAL",
    "keyboard": "KEYBOARD",
    "emergency": "EBRAKE",
    "none": "NO_COMMAND",
}


def control_state(active_input, dead_man):
    """AUTONOMY / STOPPED / MANUAL / KEYBOARD / EBRAKE / NO_COMMAND (see ControlStatus.msg)."""
    if active_input == "autonomy":
        return "AUTONOMY" if dead_man else "STOPPED"
    return MUX_STATES.get(active_input or "none", active_input.upper())
