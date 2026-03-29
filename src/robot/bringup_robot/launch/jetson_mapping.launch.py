# MIT License

# Copyright (c) 2025 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

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
    vesc_config = os.path.join(
        get_package_share_directory('bringup_robot'),
        'config',
        'vesc.yaml'
    )
    sensors_config = os.path.join(
        get_package_share_directory('bringup_robot'),
        'config',
        'sensors.yaml'
    )
    joy_teleop_config = os.path.join(
       get_package_share_directory('bringup_robot'),
       'config',
       'joy_teleop.yaml'
    )

    mux_config = os.path.join(
        get_package_share_directory('bringup_robot'),
        'config',
        'mux.yaml'
    )
    
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Descriptions for sensor configs')
    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')

    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')
    
    ld = LaunchDescription([vesc_la, sensors_la, joy_la, mux_la])

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
    
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    
    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('sensors_config')]
    )
    
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    
    #add the teleop stuff 
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
    
    ekf = Node (
        package="ackermann_ekf",
        executable="ekf",
        name ="ekf",
        output="screen"
    )    
    
    #add the rviz node 
    rviz2_node = Node (
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', os.path.join(get_package_share_directory('bringup_robot'), 'config/rviz', 'mapping.rviz')]
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'use_odometry': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'use_sim_time' : False,
        }]
    )
    

    # finalize
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    # ld.add_action(throttle_interpolator_node)
    ld.add_action(urg_node)
    ld.add_action(static_tf_node)
    ld.add_action(joy)
    ld.add_action(gamepad)
    ld.add_action(ackermann_mux_node)
    ld.add_action(ebreak)
    ld.add_action(slam_node)
    ld.add_action(rviz2_node)
    ld.add_action(ekf)

    return ld