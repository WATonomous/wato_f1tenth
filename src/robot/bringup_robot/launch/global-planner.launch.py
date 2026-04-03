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
    
    mux_config = os.path.join(
        get_package_share_directory('bringup_robot'),
        'config/sim',
        'mux.yaml'
    )
    
    gamepad_config = os.path.join(
        get_package_share_directory('teleop_utils'),
        'config',
        'joypad_config.yaml'
    )

    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')
    
    gamepad_la = DeclareLaunchArgument (
        'gamepad_config',
        default_value=gamepad_config,
        description='the config with gamepad settings'
    )

    ld = LaunchDescription([gamepad_la, mux_la]) # Begin building a launch description
  
    # the first 4 nodes are MANDATORY to launch every time
    # they are responsible for providing you with odometry
    # and emulating the transfrom that will be present on the 
    # actual car during racing and localizing 
    
    # NOTE : 
    # this odometry is perfect, that is not the case on 
    # the real car. so don't expect to get the same performance 
    # on real car, as the real odom drifts over time
   
    # provides position, orientation and linear speed 
    fake_odom = Node (
        package='sim_util',
        name='fake_odom_node',
        executable='fake_odom_node',
    )
    
    ld.add_action(fake_odom)
   
    # remaps laser scan to new topic, 
    # some times nesscary 
    scan_remap = Node (
        package='sim_util',
        name='scan_remap_node',
        executable='scan_remap_node',
    )
    
    ld.add_action(scan_remap)

    #provies a tranfrom form the laser scaner to 
    # to the car / base_link
    laser_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    
    ld.add_action(laser_to_base_link)
   
    # provides a transfrom map to car
    # this emulates how particle filter 
    # functions on the real car 
    map_to_odom = Node (
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )
    
    ld.add_action(map_to_odom)
    
    # the job the conterter is to provide 
    # a way to convert velocity and steering comands
    # that are compatible with the real car to 
    # steering and throtel comands that work with 
    # sim THIS IS WHY THIS MUST ALWAYS BE ACTIVE
    
    ackermann_converter = Node (
        package='sim_util',
        name='ackermann_converter',
        executable='ackermann_converter_node',
    )
    
    ld.add_action(ackermann_converter)

    # provies a way to prioritize inputs at different leves
    # for example, you would want your emergency breaking system
    # to take a higher priority over your teleop or autonomoues comands
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
    )     
    
    ld.add_action(ackermann_mux_node)
    
    #creats a topic which will have controler input
    joy = Node (
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {'deadzone': 0.080},
            {'sticky_buttons': True}
        ]
    )

    ld.add_action(joy)

    gamepad = Node (
        package='teleop_utils',
        executable='joypad_node',
        name='gamepad_node',
        parameters=[LaunchConfiguration('gamepad_config')],
        output='screen'
    )

    ld.add_action(gamepad)

    global_planner = Node (
        package='global_planner',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen'
    )
    
    ld.add_action(global_planner)

    #slam_node = Node(
        #package='slam_toolbox',
        #executable='async_slam_toolbox_node',
        #name='slam_toolbox',
        #output='screen',
        #parameters=[{
            #'scan_topic': '/scan',
            #'use_odometry': True,
            #'map_frame': 'map',
            #'odom_frame': 'odom',
            #'base_frame': 'base_link',
            #'use_sim_time' : True,
        #}]
    #)
    
    #ld.add_action(slam_node)
     
    return ld
