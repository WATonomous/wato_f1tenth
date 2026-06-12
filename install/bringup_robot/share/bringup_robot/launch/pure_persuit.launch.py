# README
# Starts the pure pursuit controller and local planning stack. The base robot
# launch is still responsible for localization, global planning, muxing, and costmap.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


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
        'hybrid_astar_planner.yaml'
    )

    racing_line_file = os.path.join(
        get_package_share_directory('global_planner'),
        'assets',
        'e7_fifth_v3_optimal.csv'
    )

    pure_persuit_la = DeclareLaunchArgument(
        'pure_persuit_config',
        default_value=pure_persuit_config,
        description='Pure pursuit configuration file'
    )

    local_planning_la = DeclareLaunchArgument(
        'local_planning_config',
        default_value=local_planning_config,
        description='Local planning configuration file'
    )

    racing_line_la = DeclareLaunchArgument(
        'racing_line_file',
        default_value=racing_line_file,
        description='CSV racing line used by state manager and local planner'
    )

    pure_persuit = Node(
        package='pure_persuit',
        executable='pure_persuit_node',
        name='pure_persuit_node',
        parameters=[LaunchConfiguration('pure_persuit_config')],
        output='screen'
    )

    occupancy_grid_frame_adapter = Node(
        package='local_planning',
        executable='occupancy_grid_frame_adapter_node',
        name='occupancy_grid_frame_adapter_node',
        parameters=[{
            'raw_grid_topic': '/costmap',
            'grid_topic': '/occupancy_grid',
            'frame_id': 'map',
        }],
        output='screen'
    )

    state_manager = Node(
        package='local_planning',
        executable='state_manager_node',
        name='state_manager_node',
        parameters=[
            LaunchConfiguration('local_planning_config'),
            {'racing_line_file': LaunchConfiguration('racing_line_file')},
        ],
        remappings=[
            ('/odom', '/pf/pose/odom'),
        ],
        output='screen'
    )

    local_planning = Node(
        package='local_planning',
        executable='hybrid_astar_planner_node',
        name='hybrid_astar_planner_node',
        parameters=[
            LaunchConfiguration('local_planning_config'),
            {'racing_line_file': LaunchConfiguration('racing_line_file')},
        ],
        remappings=[
            ('/odom', '/pf/pose/odom'),
            ('/path', '/local_path'),
        ],
        output='screen'
    )

    return LaunchDescription([
        pure_persuit_la,
        local_planning_la,
        racing_line_la,
        occupancy_grid_frame_adapter,
        state_manager,
        local_planning,
        pure_persuit,
    ])
