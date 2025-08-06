from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():

    ld = LaunchDescription() # Begin building a launch description

    deadzone = DeclareLaunchArgument (
        'deadzone',
        default_value='0.080',
        description='the deadzone sensitivity of the joystick'
    )

    ld.add_action(deadzone)

    deadzone_value = LaunchConfiguration('deadzone')


    #creats a topic which will have controler input
    joy = Node (
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {'deadzone': deadzone_value}
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
