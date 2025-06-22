from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    
    ld = LaunchDescription() # Begin building a launch description
    
    world_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_tf',
        arguments = ['0.000', '0.000', '0.000', # (X, Y, Z) translation
                        '0.000', '0.000', '0.000', # (Yaw, Pitch, Roll) body-fixed axis rotation
                        'world', 'map']
    )

    world_to_odom_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom_tf',
            arguments = ['0.74', '3.16', '0.06', # (X, Y, Z) translation
                         '4.71', '0.03', '0.00', # (Yaw, Pitch, Roll) body-fixed axis rotation
                         'world', 'odom']
    )

    # laider_to_f1est = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='laider_to_f1est',
    #         arguments = ['0.2733', '0.0', '0.096', # (X, Y, Z) translation
    #                      '0.000', '0.000', '0.000', # (Yaw, Pitch, Roll) body-fixed axis rotation
    #                      'f1_est', 'lidar']
    # )
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        emulate_tty=True,
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('bringup_robot'), 'maps', 'porto.yaml')},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )
    
    nav_amcl_node = Node (
        package='nav2_amcl',
        executable='amcl',
        name='nav2_amcl_node',
        output='screen',
        parameters=[{'use_sim_time' : True},
                    {'base_frame_id': 'f1_est'},
                    {'scan_topic' : '/autodrive/f1tenth_1/lidar'},
                    {'odom_frame_id' :'odom'},
                    {'Pose2D':'{x: 0.7412, y: 3.1583, z: 0.0, yaw:-1.570796327 }'}]
    )
    
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server', 'nav2_amcl_node']}]
    )
    
    
    rviz2_node = Node (
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', os.path.join(get_package_share_directory('bringup_robot'), 'config/rviz', 'localize.rviz')]
    )

    # wheel_odom_node = Node (
    #     package="wheel_odom",
    #     executable="wheel_odom_node",
    #     name ="wheel_odom",
    #     output="screen"
    # )
    
    # ekf_node = Node (
    #     package="ackermann_ekf",
    #     executable="ekf_node",
    #     name="ekf_node",
    #     output="screen"
    # )
    
    # transfrom_publisher = Node (
    #     package="transform_broadcast",
    #     executable="transform_broadcast_node",
    #     name="transform_broadcast_node",
    #     output="screen"
    # )
    


    
    ld.add_action(world_to_map_tf)
    ld.add_action(world_to_odom_tf)
    ld.add_action(map_server_node)
    # ld.add_action(laider_to_f1est)
    ld.add_action(nav_amcl_node)
    # ld.add_action(wheel_odom_node)
    ld.add_action(nav_lifecycle_node)
    # ld.add_action(ekf_node)
    # ld.add_action(transfrom_publisher)
    ld.add_action(rviz2_node)  

    return ld
