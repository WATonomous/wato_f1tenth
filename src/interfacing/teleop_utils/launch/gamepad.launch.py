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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    gamepad_config = os.path.join(
        get_package_share_directory("teleop_utils"), "config", "joypad_config.yaml"
    )

    gamepad_la = DeclareLaunchArgument(
        "gamepad_config",
        default_value=gamepad_config,
        description="the config with gamepad settings",
    )

    ld = LaunchDescription([gamepad_la])  # Begin building a launch description

    deadzone = DeclareLaunchArgument(
        "deadzone",
        default_value="0.080",
        description="the deadzone sensitivity of the joystick",
    )

    ld.add_action(deadzone)

    sticky_button_toggle = DeclareLaunchArgument(
        "sticky_button",
        default_value="True",
        description="wheather the button are toggles",
    )

    ld.add_action(sticky_button_toggle)

    deadzone_value = LaunchConfiguration("deadzone")
    sticky_button_value = LaunchConfiguration("sticky_button")

    # creats a topic which will have controler input
    joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {"deadzone": deadzone_value},
            {"sticky_buttons": sticky_button_value},
        ],
    )

    ld.add_action(joy)

    gamepad = Node(
        package="teleop_utils",
        executable="joypad_node",
        name="gamepad_node",
        parameters=[LaunchConfiguration("gamepad_config")],
        output="screen",
    )

    ld.add_action(gamepad)

    return ld
