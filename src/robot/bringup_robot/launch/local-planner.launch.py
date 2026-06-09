from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup_robot')
    global_planner_share = get_package_share_directory('global_planner')
    local_planning_share = get_package_share_directory('local_planning')
    costmap_share = get_package_share_directory('costmap')

    minimum_launch = os.path.join(bringup_share, 'launch', 'minimumEx.launch.py')
    mux_config = os.path.join(bringup_share, 'config', 'sim', 'mux.yaml')
    pure_persuit_config = os.path.join(
        bringup_share, 'config', 'pure_persuit', 'pure_persuit.yaml')
    planner_config = os.path.join(
        local_planning_share, 'config', 'hybrid_astar_planner.yaml')
    racing_line_file = os.path.join(
        global_planner_share, 'assets', 'optmial_clean_map.csv')
    costmap_param_file = os.path.join(costmap_share, 'config', 'params.yaml')

    mux_config_arg = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Ackermann mux configuration file')
    pure_persuit_config_arg = DeclareLaunchArgument(
        'pure_persuit_config',
        default_value=pure_persuit_config,
        description='Pure pursuit configuration file')
    planner_config_arg = DeclareLaunchArgument(
        'planner_config',
        default_value=planner_config,
        description='Local Frenet lattice planner configuration file')
    racing_line_file_arg = DeclareLaunchArgument(
        'racing_line_file',
        default_value=racing_line_file,
        description='CSV racing line used by global planner, state manager, and local planner')
    costmap_param_file_arg = DeclareLaunchArgument(
        'costmap_param_file',
        default_value=costmap_param_file,
        description='Path to config file for costmap node')
    debug_path_topic_arg = DeclareLaunchArgument(
        'debug_path_topic',
        default_value='/local_path_map',
        description='Map-frame debug topic for the selected local plan')

    minimum_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(minimum_launch),
        launch_arguments={
            'mux_config': LaunchConfiguration('mux_config'),
        }.items())

    global_planner = Node(
        package='global_planner',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[{
            'file_directory': '/assets/optmial_clean_map.csv',
            'waypoint_frame_id': 'map',
        }])

    costmap = Node(
        package='costmap',
        executable='costmap_node',
        name='occupancy_grid_generator',
        output='screen',
        parameters=[LaunchConfiguration('costmap_param_file')])

    occupancy_grid_adapter = Node(
        package='local_planning',
        executable='occupancy_grid_frame_adapter_node',
        name='occupancy_grid_frame_adapter_node',
        output='screen',
        parameters=[{'raw_grid_topic': '/costmap'}])

    local_planner = Node(
        package='local_planning',
        executable='hybrid_astar_planner_node',
        name='hybrid_astar_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('planner_config'),
            {'racing_line_file': LaunchConfiguration('racing_line_file')},
            {'debug_path_topic': LaunchConfiguration('debug_path_topic')},
        ],
        remappings=[('/path', '/local_path')])

    state_manager = Node(
        package='local_planning',
        executable='state_manager_node',
        name='state_manager_node',
        output='screen',
        parameters=[
            LaunchConfiguration('planner_config'),
            {'racing_line_file': LaunchConfiguration('racing_line_file')},
        ])

    pure_persuit = Node(
        package='pure_persuit',
        executable='pure_persuit_node',
        name='pure_persuit_node',
        output='screen',
        parameters=[LaunchConfiguration('pure_persuit_config')])

    return LaunchDescription([
        mux_config_arg,
        pure_persuit_config_arg,
        planner_config_arg,
        racing_line_file_arg,
        costmap_param_file_arg,
        debug_path_topic_arg,
        minimum_stack,
        global_planner,
        costmap,
        occupancy_grid_adapter,
        local_planner,
        state_manager,
        pure_persuit,
    ])
