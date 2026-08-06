# Launches the obstacle injector only.  Splice it into the chain by pointing
# the frame adapter at the injected topic:
#
#   ros2 launch costmap_test_tools injected_costmap.launch.py
#   ros2 launch local_planning local-planner.launch.py raw_grid_topic:=/costmap_injected
#
# Test-only.  Never include this from a real-car launch.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('costmap_test_tools'),
        'config',
        'scenarios.yaml'
    )

    scenario_config_la = DeclareLaunchArgument(
        'scenario_config',
        default_value=default_config,
        description='Obstacle scenario definitions for the costmap injector'
    )

    injector = Node(
        package='costmap_test_tools',
        executable='costmap_obstacle_injector_node',
        name='costmap_obstacle_injector_node',
        parameters=[LaunchConfiguration('scenario_config')],
        output='screen'
    )

    return LaunchDescription([scenario_config_la, injector])
