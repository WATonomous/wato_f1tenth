from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    config = os.path.join(get_package_share_directory('bringup_robot'), 'config/simulator', 'perception.yaml')
    map_name = yaml.safe_load(open(config, 'r'))['map_server']['ros__parameters']['map']


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
    
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    particle_filter_node = Node(
        package='particle_filter',
        executable='particle_filter',
        name='particle_filter',
        parameters=[config]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('bringup_robot'), 'config/rviz', 'localize.rviz')]
    )

    ld = LaunchDescription()
    ld.add_action(world_to_map_tf)
    ld.add_action(world_to_odom_tf)
    ld.add_action(particle_filter_node)
    ld.add_action(map_server_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(rviz_node)
    return ld