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
        get_package_share_directory('pure_persuit'),
        'config',
        'pure_persuit.yaml'
    )

    pure_persuit_la = DeclareLaunchArgument (
        'pure_persuit_config',
        default_value=pure_persuit_config,
        description='the config for pure persuit settings'
    )

    ld = LaunchDescription([pure_persuit_la]) # Begin building a launch description

    pure_persuit = Node (
        package='pure_persuit',
        executable='pure_persuit_node',
        name='pure_persuit_node',
        parameters=[LaunchConfiguration('pure_persuit_config')],
        output='screen'
    )

    ld.add_action(pure_persuit)

    return ld