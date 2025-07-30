from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    localize_config = os.path.join(
        get_package_share_directory('bringup_robot'),
        'config/simulator',
        'localize.yaml'
    )

    localize_config_dict = yaml.safe_load(open(localize_config, 'r'))

    map_name = localize_config_dict['map_server']['ros__parameters']['map']
    
    localize_la = DeclareLaunchArgument(
        'localize_config',
        default_value=localize_config,
        description='Localization configs'
    )

    ld = LaunchDescription([localize_la])

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

    pf_node = Node(
        package='particle_filter',
        executable='particle_filter',
        name='particle_filter',
        parameters=[LaunchConfiguration('localize_config')]
    )

    ld.add_action(pf_node)

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

    ld.add_action(map_server_node)

    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
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