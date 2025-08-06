from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    
    ld = LaunchDescription() # Begin building a launch description
    
    #add the rviz node 
    rviz2_node = Node (
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', os.path.join(get_package_share_directory('bringup_robot'), 'config/rviz', 'mapping.rviz')]
    )

    ld.add_action(rviz2_node)

    scan_remap = Node (
        package="scan_remap",
        executable="scan_remap_node",
        name ="scan_remap_node",
        output="screen"
    )

    ld.add_action(scan_remap)

    laider_tranform = Node (
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_scaner_tf',
        arguments = ['0.2733', '0.000', '0.096', # (X, Y, Z) translation
                     '0.000', '0.000', '0.000', # (Yaw, Pitch, Roll) body-fixed axis rotation
                     'base_link', 'laser_scaner']
    )

    ld.add_action(laider_tranform)

    wheel_odom_node = Node (
        package="wheel_odom",
        executable="wheel_odom_node",
        name ="wheel_odom",
        output="screen"
    )

    ld.add_action(wheel_odom_node)
    
    ekf_node = Node (
        package="ackermann_ekf",
        executable="ekf_node",
        name="ekf_node",
        output="screen"
    )
    
    ld.add_action(ekf_node)
    
    transfrom_publisher = Node (
        package="transform_broadcast",
        executable="transform_broadcast_node",
        name="transform_broadcast_node",
        output="screen"
    )
    
    ld.add_action(transfrom_publisher)
     
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
            'use_sim_time' : True,
        }]
    )
    
    ld.add_action(slam_node)
    

    return ld