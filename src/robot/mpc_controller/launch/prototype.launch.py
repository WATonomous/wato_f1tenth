"""Deterministic matching-model simulation; autonomy requires an explicit enable."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('mpc_controller')
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="solo", choices=["solo", "obstacle", "blocked"]),
        DeclareLaunchArgument('visualization', default_value='true', choices=['true', 'false']),
        Node(package='ackermann_mux', executable='ackermann_mux',
             parameters=[os.path.join(share, 'config', 'mux.yaml')]),
        Node(package='mpc_controller', executable='mpc_node', output='screen',
             parameters=[{'publish_visualization': ParameterValue(LaunchConfiguration('visualization'), value_type=bool)}]),
        Node(package='mpc_controller', executable='fixture_simulator', output='screen',
             parameters=[{'scene': LaunchConfiguration('scene')}]),
    ])
