"""Standalone launch for region_manager — point at any params yaml you want.

By default reads the region_ICRA_Masters.yaml in mppi_bringup. Override at
launch with:
    ros2 launch region_manager region_manager.launch.py \
        config_file:=/abs/path/to/region_config.yaml

Run this alongside your usual mppi launch (no changes there); region_manager
publishes /region/active which the particle_filter on the bridge-simple branch
subscribes to.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    region_manager = Node(
        package='region_manager',
        executable='region_manager',
        name='region_manager',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mppi_bringup'),
                'config',
                'region_ICRA_Masters.yaml',
            ]),
            description='Yaml with region_manager params (bubble centres, radii, pose topic).',
        ),
        region_manager,
    ])
