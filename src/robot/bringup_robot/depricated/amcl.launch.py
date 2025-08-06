from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    ld = LaunchDescription()

    bringup_share_dir = get_package_share_directory('bringup_robot')

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

    map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[{
        'yaml_filename': os.path.join(bringup_share_dir, 'maps', 'theMap.yaml'),
        'topic': 'map',
        'frame_id': 'map',
        'use_sim_time': True
    }]
)

    ld.add_action(map_server_node)

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(bringup_share_dir, 'config', 'simulator', 'amcl.yaml')]
    )

    ld.add_action(amcl_node)

    nav_lifecycle_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_localization',
    output='screen',
    parameters=[{
        'use_sim_time': True,
        'autostart': True,
        'node_names': ['map_server', 'amcl']
    }]
)

    ld.add_action(nav_lifecycle_node)

    rviz2_node = Node (
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', os.path.join(get_package_share_directory('bringup_robot'), 'config/rviz', 'mylocalize.rviz')]
    )

    ld.add_action(rviz2_node)
    
    return ld