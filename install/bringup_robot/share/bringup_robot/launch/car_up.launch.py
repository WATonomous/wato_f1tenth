from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mux_config = os.path.join(
        get_package_share_directory('bringup_robot'),
        'config',
        'mux.yaml'
    )
    
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')
    
    ld = LaunchDescription([mux_la])
    
    deadzone = DeclareLaunchArgument (
        'deadzone',
        default_value='0.080',
        description='the deadzone sensitivity of the joystick'
    )
    
    sticky_button_toggle = DeclareLaunchArgument (
        'sticky_button',
        default_value='True',
        description='wheather the button are toggles'
    )
    
    ld.add_action(deadzone)
    ld.add_action(sticky_button_toggle)
    
    deadzone_value = LaunchConfiguration('deadzone')
    sticky_button_value = LaunchConfiguration('sticky_button')
    
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
    
    gamepad = Node (
        package='gamepad',
        executable='joypad_node',
        name='joypad_node',
        output='screen'
    )
    
    ackerman_converter = Node (
        package='ackermann_conterter',
        executable='ackermann_converter',
        name='ackermann_converter',
        output='screen',
    )
    
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        #remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )    
    
    ebreak = Node (
        package='safety_node',
        executable='ebreak',
        name='ebreak',
        output='screen',
    )
        
    
    ld.add_action(joy)
    ld.add_action(gamepad)
    ld.add_action(ackerman_converter)
    ld.add_action(ackermann_mux_node)
    ld.add_action(ebreak)
    
    return ld