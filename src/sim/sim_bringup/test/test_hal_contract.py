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
"""Contract test: replay a recorded AutoDRIVE bag through the real sim adapter
(sim.launch.yaml without the bridge) and check the /hal sensor topics with hal_contract.

Re-record test/data/autodrive_contract only if AutoDRIVE's topics change
(agents/workflows/sim-to-car.md).
"""

import os
import unittest

import launch_testing
import launch_testing.asserts
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

# Keep the replay off the network and away from a live stack on the same machine
os.environ["ROS_DOMAIN_ID"] = "87"
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

BAG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "data", "autodrive_contract"
)


def generate_test_description():
    sim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sim_bringup"), "launch", "sim.launch.yaml"
            )
        ),
        launch_arguments={
            "bridge": "false",
            "localization": "truth",
            "mapping": "false",
        }.items(),
    )
    # base_link -> laser comes from interfacing, which this test doesn't start
    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.27",
            "--z",
            "0.11",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "laser",
        ],
    )
    replay = ExecuteProcess(cmd=["ros2", "bag", "play", BAG, "--loop"], output="screen")
    checker = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "hal_contract",
            "check",
            "--platform",
            "sim",
            "--group",
            "sensors",
            "--warmup",
            "3",
        ],
        output="screen",
    )
    return (
        LaunchDescription(
            [
                sim,
                laser_tf,
                replay,
                TimerAction(period=2.0, actions=[checker]),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"checker": checker},
    )


class TestContract(unittest.TestCase):
    def test_checker_finishes(self, proc_info, checker):
        proc_info.assertWaitForShutdown(process=checker, timeout=60)


@launch_testing.post_shutdown_test()
class TestContractResult(unittest.TestCase):
    def test_contract_holds(self, proc_info, checker):
        launch_testing.asserts.assertExitCodes(proc_info, process=checker)
