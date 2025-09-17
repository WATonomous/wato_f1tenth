from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

import os

def generate_launch_description():

    ld = LaunchDescription() # Begin building a launch description

    deadzone = DeclareLaunchArgument (
        'deadzone',
        default_value='0.080',
        description='the deadzone sensitivity of the joystick'
    )

    ld.add_action(deadzone)
    
    sticky_button_toggle = DeclareLaunchArgument (
        'sticky_button',
        default_value='True',
        description='wheather the button are toggles'
    )
    
    ld.add_action(sticky_button_toggle)

    deadzone_value = LaunchConfiguration('deadzone')
    sticky_button_value = LaunchConfiguration('sticky_button')


    #creats a topic which will have controler input
    joy = Node (
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {'deadzone': deadzone_value},
            {'sticky_buttons': sticky_button_value}
        ]
    )

    ld.add_action(joy)

    gamepad = Node (
        package='gamepad',
        executable='joypad_node',
        name='joypad_node',
        output='screen'
    )

    ld.add_action(gamepad)
    
    #ackerman translation layer
    ackermann_converter = Node (
        package='ackermann_conterter',
        executable='ackermann_converter',
        name='ackermann_converter',
        output='screen'
    )
    
    ld.add_action(ackermann_converter)
    
    wheel_odom_node = Node (
        package="wheel_odom",
        executable="odom",
        name ="wheel_odom",
        output="screen"
    )

    ld.add_action(wheel_odom_node)
    
    # ekf 
    
    ekf = TimerAction(
        period=3.0,  # delay in seconds
        actions=[
            Node(
                package="ackermann_ekf",
                executable="ekf",
                name="ekf",
                output="screen"
            )
        ]
    )
    #ekf = Node (
        #package="ackermann_ekf",
        #executable="ekf",
        #name ="ekf",
        ##output="screen"
    #)
    
    ld.add_action(ekf)
    
    test_data = Node (
        package="test_pakcage",
        executable="test_node",
        name ="test_node",
        output="screen"
    )
    
    ld.add_action(test_data)

    return ld
