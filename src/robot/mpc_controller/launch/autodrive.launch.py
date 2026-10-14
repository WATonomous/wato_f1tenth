"""External AutoDRIVE bridge. Requires a reference matched to its world frame."""
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
        DeclareLaunchArgument('reference_asset',
                              description='Map-matched CSV under global_planner share, e.g. /assets/map_reference.csv'),
        DeclareLaunchArgument('laser_x_m', description='Verified simulator base_link to lidar x offset'),
        DeclareLaunchArgument('rear_axle_offset_x_m',
                              description='Verified simulator pose to rear axle x offset'),
        Node(package='global_planner', executable='global_planner_node',
             parameters=[{'file_directory': LaunchConfiguration('reference_asset'),
                          'waypoint_frame_id': 'map'}]),
        Node(package='sim_util', executable='fake_odom_node', parameters=[{'header_frame': 'map'}]),
        Node(package='sim_util', executable='scan_remap_node'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', LaunchConfiguration('laser_x_m'), '--frame-id', 'base_link',
                        '--child-frame-id', 'laser']),
        Node(package='costmap', executable='costmap_node',
             parameters=[os.path.join(get_package_share_directory('costmap'), 'config', 'params.yaml')]),
        Node(package='ackermann_mux', executable='ackermann_mux',
             parameters=[os.path.join(share, 'config', 'mux.yaml')]),
        Node(package='mpc_controller', executable='mpc_node', output='screen',
             parameters=[{'rear_axle_offset_x_m': ParameterValue(LaunchConfiguration('rear_axle_offset_x_m'), value_type=float)}]),
        Node(package='sim_util', executable='ackermann_converter_node'),
    ])
