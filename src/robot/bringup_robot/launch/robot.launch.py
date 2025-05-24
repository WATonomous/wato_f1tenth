from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():

    ld = LaunchDescription()  # Begin building a launch description

    #################### Costmap Node #####################

    gym_vis_node = Node(
        package='gym_vis',
        name='gym_vis_node',
        executable='gym_vis_node',
    )

    ld.add_action(gym_vis_node)

    imu_throttle_node = Node(
        package='imu_throttle',
        name='imu_throttle_node',
        executable='imu_throttle_node',
    )
    ld.add_action(imu_throttle_node)
# #################### Example Node #####################

# name1_node = Node(
#     package='package_name',
#     name='name_node',
#     executable='name_node',
# )
# ld.add_action(name1_node)

# #################### Costmap Node #####################

# costmap_pkg_prefix = get_package_share_directory('costmap')
# costmap_param_file = os.path.join(
#     costmap_pkg_prefix, 'config', 'params.yaml')

# costmap_param = DeclareLaunchArgument(
#     'costmap_param_file',
#     default_value=costmap_param_file,
#     description='Path to config file for producer node'
# )
# costmap_node = Node(
#     package='costmap',
#     name='costmap_node',
#     executable='costmap_node',
#     parameters=[LaunchConfiguration('costmap_param_file')],
# )
# ld.add_action(costmap_param)
# ld.add_action(costmap_node)

# #################### Map Memory Node #####################
# map_memory_pkg_prefix = get_package_share_directory('map_memory')
# map_memory_param_file = os.path.join(
#     map_memory_pkg_prefix, 'config', 'params.yaml')

# map_memory_param = DeclareLaunchArgument(
#     'map_memory_param_file',
#     default_value=map_memory_param_file,
#     description='Path to config file for producer node'
# )
# map_memory_node = Node(
#     package='map_memory',
#     name='map_memory_node',
#     executable='map_memory_node',
#     parameters=[LaunchConfiguration('map_memory_param_file')],
# )
# ld.add_action(map_memory_param)
# ld.add_action(map_memory_node)


return ld
