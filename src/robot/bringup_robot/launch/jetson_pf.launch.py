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
import yaml
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

    localize_config = os.path.join(
        get_package_share_directory('bringup_robot'),
        'config/simulator',
        'localize.yaml'
    )

    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')
    
    localize_config_dict = yaml.safe_load(open(localize_config, 'r'))

    map_name = localize_config_dict['map_server']['ros__parameters']['map']
    
    localize_la = DeclareLaunchArgument(
        'localize_config',
        default_value=localize_config,
        description='Localization configs'
    )

    ld = LaunchDescription([vesc_la, sensors_la, joy_la, localize_la, mux_la])

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
    
    #particle filter minimum suff
    pf_node = Node(
        package='particle_filter',
        executable='particle_filter',
        name='particle_filter',
        parameters=[LaunchConfiguration('localize_config')]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('bringup_robot'), 'maps', map_name + '.yaml')},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )

    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )
    
    #riv for vis
    rviz2_node = Node (
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', os.path.join(get_package_share_directory('bringup_robot'), 'config/rviz', 'mylocalize.rviz')]
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
    # vesc drivers and odom stuff
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(urg_node)
    ld.add_action(static_tf_node)
    # this space for ekf
    ld.add_action(rviz2_node)
    #teleop stuff
    ld.add_action(joy)
    ld.add_action(gamepad)
    ld.add_action(ackermann_mux_node)
    ld.add_action(ebreak)
    #pf suff
    ld.add_action(pf_node)
    ld.add_action(map_server_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(ekf)
    
    return ld