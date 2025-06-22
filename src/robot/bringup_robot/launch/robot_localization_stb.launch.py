from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    
    ld = LaunchDescription() # Begin building a launch description
    
    rviz2_node = Node (
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    ld.add_action(rviz2_node)

    wheel_odom_node = Node (
        package="wheel_odom",
        executable="wheel_odom_node",
        name ="wheel_odom",
        output="screen"
    )

    ld.add_action(wheel_odom_node)
    
    static_transforms = [
        (0.0, 0.118, 0.0, 0.0, 0.0, 0.0, 'left_encoder_est'),
        (0.0, -0.118, 0.0, 0.0, 0.0, 0.0, 'right_encoder_est'),
        (0.08, 0.0, 0.055, 0.0, 0.0, 0.0, 'ips_est'),
        (0.08, 0.0, 0.055, 0.0, 0.0, 0.0, 'imu_est'),
        (0.2733, 0.0, 0.096, 0.0, 0.0, 0.0, 'lidar_est'),
        (-0.015, 0.0, 0.15, 0.0, 10.0, 0.0, 'front_camera_est'),
        (0.33, 0.118, 0.0, 0.0, 0.0, 0.0, 'front_left_wheel_est'),
        (0.33, -0.118, 0.0, 0.0, 0.0, 0.0, 'front_right_wheel_est'),
        (0.0, 0.118, 0.0, 0.0, 0.0, 0.0, 'rear_left_wheel_est'),
        (0.0, -0.118, 0.0, 0.0, 0.0, 0.0, 'rear_right_wheel_est'),
    ]

    for x, y, z, roll, pitch, yaw, child in static_transforms:
        ld.add_action(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_{child}',
                arguments=[
                    str(x), str(y), str(z),
                    str(roll), str(pitch), str(yaw),
                    'f1_est', child
                ]
            )
        )
     

    #################### Costmap Node #####################

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'scan_topic': '/autodrive/f1tenth_1/lidar',
            'use_odometry': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'f1_est',
            'use_sim_time' : True,
            'mode' : 'locallization',
            'map_file_name' : '/home/bolty/ament_ws/src/robot/bringup_robot/map_files/map1_seria',
            'map_start_at_dock' : True
        }]
    )
    
    ld.add_action(slam_node)

    # #################### Example Node #####################

    # name1_node = Node(
    #     package='package_name',
    #     name='name_node',
    #     executable='name_node',
    # )
    # ld.add_action(name1_node)

    # #################### Costmap Node #####################

    # costmap_pkg_prefix = get_package_share_directory('costmap')
    # costmap_param_file = os.path.join(
    #     costmap_pkg_prefix, 'config', 'params.yaml')
    
    # costmap_param = DeclareLaunchArgument(
    #     'costmap_param_file',
    #     default_value=costmap_param_file,
    #     description='Path to config file for producer node'
    # )
    # costmap_node = Node(
    #     package='costmap',
    #     name='costmap_node',
    #     executable='costmap_node',
    #     parameters=[LaunchConfiguration('costmap_param_file')],
    # )
    # ld.add_action(costmap_param)
    # ld.add_action(costmap_node)

    # #################### Map Memory Node #####################
    # map_memory_pkg_prefix = get_package_share_directory('map_memory')
    # map_memory_param_file = os.path.join(
    #     map_memory_pkg_prefix, 'config', 'params.yaml')
    
    # map_memory_param = DeclareLaunchArgument(
    #     'map_memory_param_file',
    #     default_value=map_memory_param_file,
    #     description='Path to config file for producer node'
    # )
    # map_memory_node = Node(
    #     package='map_memory',
    #     name='map_memory_node',
    #     executable='map_memory_node',
    #     parameters=[LaunchConfiguration('map_memory_param_file')],
    # )
    # ld.add_action(map_memory_param)
    # ld.add_action(map_memory_node)
    

    return ld
