# README 
# this is the minimum launch file nesscary 
# nodes required to make emulate the localization stack
# and provide handling for control inputs and 
# multiplexing control inputs

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    
    pure_persuit_config = os.path.join(
        get_package_share_directory('bringup_robot'),
        'config/pure_persuit',
        'pure_persuit.yaml'
    )

    pure_persuit_la = DeclareLaunchArgument (
        'pure_persuit_config',
        default_value=pure_persuit_config,
        description='the config for pure persuit settings'
    )
    
    local_planning_config = os.path.join(
        get_package_share_directory('local_planning'),
        'config',
        'hybrid_astar_planner.yaml'
    )

    local_planning_la = DeclareLaunchArgument(
        'local_planning_config',
        default_value=local_planning_config,
        description='the config for local planning settings'
    )

    ld = LaunchDescription([pure_persuit_la, local_planning_la]) # Begin building a launch description
     
    pure_persuit = Node (
        package='pure_persuit',
        executable='pure_persuit_node',
        name='pure_persuit_node',
        parameters=[LaunchConfiguration('pure_persuit_config')],
        output='screen'
    )

    state_manager = Node(
        package='local_planning',
        executable='state_manager_node',
        name='state_manager_node',
        parameters=[LaunchConfiguration('local_planning_config')],
        remappings=[
            ('/odom', '/pf/pose/odom'),
        ],
        output='screen'
    )

    local_planning = Node(
        package='local_planning',
        executable='hybrid_astar_planner_node',
        name='hybrid_astar_planner_node',
        parameters=[LaunchConfiguration('local_planning_config')],
        remappings=[
            ('/odom', '/pf/pose/odom'),
            ('/path', '/local_path'),
        ],
        output='screen'
    )

    ld.add_action(state_manager)
    ld.add_action(local_planning)
    ld.add_action(pure_persuit)

    return ld
