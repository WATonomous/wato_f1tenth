from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup_robot')
    global_planner_share = get_package_share_directory('global_planner')
    local_planning_share = get_package_share_directory('local_planning')

    minimum_launch = os.path.join(bringup_share, 'launch', 'minimumEx.launch.py')
    mux_config = os.path.join(bringup_share, 'config', 'sim', 'mux.yaml')
    pure_persuit_config = os.path.join(
        bringup_share, 'config', 'pure_persuit', 'pure_persuit.yaml')
    planner_config = os.path.join(
        local_planning_share, 'config', 'hybrid_astar_planner.yaml')
    racing_line_file = os.path.join(
        global_planner_share, 'assets', 'optmial_clean_map.csv')

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
    occupancy_grid_width_arg = DeclareLaunchArgument(
        'occupancy_grid_width_m',
        default_value='30.0',
        description='Width of the ego-centered occupancy grid in meters')
    occupancy_grid_height_arg = DeclareLaunchArgument(
        'occupancy_grid_height_m',
        default_value='30.0',
        description='Height of the ego-centered occupancy grid in meters')

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

    occupancy_grid = Node(
        package='local_planning',
        executable='scan_to_occupancy_grid_node',
        name='scan_to_occupancy_grid_node',
        output='screen',
        parameters=[{
            'width_m': ParameterValue(LaunchConfiguration('occupancy_grid_width_m'), value_type=float),
            'height_m': ParameterValue(LaunchConfiguration('occupancy_grid_height_m'), value_type=float),
        }],
        remappings=[('/occupancy_grid', '/occupancy_grid_raw')])

    sim_obstacles = Node(
        package='local_planning',
        executable='sim_obstacle_overlay_node',
        name='sim_obstacle_overlay_node',
        output='screen')

    local_planner = Node(
        package='local_planning',
        executable='hybrid_astar_planner_node',
        name='hybrid_astar_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('planner_config'),
            {'racing_line_file': LaunchConfiguration('racing_line_file')},
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
        occupancy_grid_width_arg,
        occupancy_grid_height_arg,
        minimum_stack,
        global_planner,
        occupancy_grid,
        sim_obstacles,
        local_planner,
        state_manager,
        pure_persuit,
    ])
