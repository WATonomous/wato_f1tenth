from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    
    ld = LaunchDescription() # Begin building a launch description
    
    #creats a topic which will have controler input
    joy = Node (
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {'deadzone': 0.080}
        ]
    )

    ld.add_action(joy)

    gamepad = Node (
        package='gamepad',
        executable='gamepad_node',
        name='gamepad_node',
        output='screen'
    )

    ld.add_action(gamepad)
    

    return ld
