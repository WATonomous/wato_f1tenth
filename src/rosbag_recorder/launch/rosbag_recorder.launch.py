from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'topics',
            default_value='[]',
            description='List of topics to record. Empty = record all.'
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='',
            description='Directory to save bags. Default = package bags/ folder.'
        ),
        Node(
            package='rosbag_recorder',
            executable='rosbag_recorder_node',
            name='rosbag_recorder_node',
            parameters=[{
                'topics': '',
                'output_dir': '',
            }],
            output='screen',
        ),
    ])