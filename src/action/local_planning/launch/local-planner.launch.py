import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('local_planning'), 'config', 'local_planner.yaml'
    )
    raw_grid_topic = DeclareLaunchArgument(
        'raw_grid_topic', default_value='/costmap'
    )
    adapter = Node(
        package='local_planning',
        executable='occupancy_grid_frame_adapter_node',
        name='occupancy_grid_frame_adapter_node',
        parameters=[config, {'raw_grid_topic': LaunchConfiguration('raw_grid_topic')}],
        output='screen',
    )
    planner = Node(
        package='local_planning',
        executable='planner_node',
        name='planner_node',
        parameters=[config],
        output='screen',
    )
    return LaunchDescription([raw_grid_topic, adapter, planner])
