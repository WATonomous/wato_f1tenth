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

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_f1tenth_1',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'f1tenth_1']
    )
    
    ld.add_action(static_tf_node)
     

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'scan_topic': '/autodrive/f1tenth_1/lidar',
            'use_odometry': False,
            'map_frame': 'map',
            'odom_frame': 'world',
            'base_frame': 'f1tenth_1',
            'use_sim_time' : True,
        }]
    )
    
    ld.add_action(slam_node)
    

    return ld
