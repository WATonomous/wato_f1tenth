"""Installed Porto AutoDRIVE scene; native rear-axle odometry and LiDAR TF."""
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
        DeclareLaunchArgument('visualization', default_value='true', choices=['true', 'false']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'map', '--child-frame-id', 'world']),
        Node(package='costmap', executable='costmap_node', parameters=[{
            'robot_frame': 'roboracer_1', 'scan_topic': '/autodrive/roboracer_1/lidar',
            'output_topic': '/costmap/lidar', 'forward_distance': 3.,
            'behind_distance': 1., 'lateral_half_width': 1.5, 'resolution': .025}]),
        Node(package='mpc_controller', executable='porto_scene.py', output='screen'),
        Node(package='ackermann_mux', executable='ackermann_mux',
             parameters=[os.path.join(share, 'config', 'mux.yaml')]),
        Node(package='mpc_controller', executable='mpc_node', output='screen',
             parameters=[{'model_profile': 'autodrive', 'vehicle_frame': 'roboracer_1',
                          'max_planar_tilt_rad': .04,
                          'publish_visualization': ParameterValue(LaunchConfiguration('visualization'), value_type=bool)}],
             remappings=[('/odom', '/autodrive/roboracer_1/odom')]),
        Node(package='sim_util', executable='ackermann_converter_node'),
    ])
