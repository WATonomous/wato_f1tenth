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
        local_planning_share, 'config', 'local_frenet_lattice_planner.yaml')
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
        description='CSV racing line used by the standalone racing-line publisher and local planner')
    racing_line_topic_arg = DeclareLaunchArgument(
        'racing_line_topic',
        default_value='/racing_line',
        description='Topic on which to publish and consume the racing line')
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

    racing_line_publisher = Node(
        package='local_planning',
        executable='racing_line_publisher_node',
        name='racing_line_publisher_node',
        output='screen',
        parameters=[{
            'racing_line_file': LaunchConfiguration('racing_line_file'),
            'racing_line_topic': LaunchConfiguration('racing_line_topic'),
            'waypoint_frame_id': 'map',
        }])

    overtake_ready_publisher = Node(
        package='local_planning',
        executable='bool_topic_publisher_node',
        name='overtake_ready_publisher_node',
        output='screen',
        parameters=[{'topic': '/overtake_ready', 'value': True, 'publish_rate_hz': 1.0}])

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
        executable='local_frenet_lattice_planner_node',
        name='local_frenet_lattice_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('planner_config'),
            {'racing_line_file': LaunchConfiguration('racing_line_file')},
            {'racing_line_topic': LaunchConfiguration('racing_line_topic')},
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
            {'racing_line_topic': LaunchConfiguration('racing_line_topic')},
        ])

    pure_persuit = Node(
        package='pure_persuit',
        executable='pure_persuit_node',
        name='pure_persuit_node',
        output='screen',
        parameters=[
            LaunchConfiguration('pure_persuit_config'),
            {'global_path_topic': LaunchConfiguration('racing_line_topic')},
            {'overtake_enable': True},
        ])

    return LaunchDescription([
        mux_config_arg,
        pure_persuit_config_arg,
        planner_config_arg,
        racing_line_file_arg,
        racing_line_topic_arg,
        costmap_param_file_arg,
        debug_path_topic_arg,
        minimum_stack,
        racing_line_publisher,
        overtake_ready_publisher,
        costmap,
        occupancy_grid_adapter,
        local_planner,
        state_manager,
        pure_persuit,
    ])
