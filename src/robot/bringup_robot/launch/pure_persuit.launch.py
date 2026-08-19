"""Start the stateful local planner and pure-pursuit controller on the car.

The base robot launch remains responsible for localization, global planning,
muxing, and publishing the real /costmap.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pure_persuit_config = os.path.join(
        get_package_share_directory('bringup_robot'),
        'config',
        'pure_persuit',
        'pure_persuit.yaml'
    )
    local_planning_config = os.path.join(
        get_package_share_directory('local_planning'),
        'config',
        'local_planner.yaml'
    )

    pure_persuit_config_argument = DeclareLaunchArgument(
        'pure_persuit_config',
        default_value=pure_persuit_config,
        description='Pure-pursuit configuration file'
    )
    local_planning_config_argument = DeclareLaunchArgument(
        'local_planning_config',
        default_value=local_planning_config,
        description='Stateful local-planner configuration file'
    )

    occupancy_grid_frame_adapter = Node(
        package='local_planning',
        executable='occupancy_grid_frame_adapter_node',
        name='occupancy_grid_frame_adapter_node',
        parameters=[LaunchConfiguration('local_planning_config')],
        output='screen'
    )
    planner = Node(
        package='local_planning',
        executable='planner_node',
        name='planner_node',
        parameters=[LaunchConfiguration('local_planning_config')],
        output='screen'
    )
    pure_persuit = Node(
        package='pure_persuit',
        executable='pure_persuit_node',
        name='pure_persuit_node',
        parameters=[LaunchConfiguration('pure_persuit_config')],
        output='screen'
    )

    return LaunchDescription([
        pure_persuit_config_argument,
        local_planning_config_argument,
        occupancy_grid_frame_adapter,
        planner,
        pure_persuit,
    ])
