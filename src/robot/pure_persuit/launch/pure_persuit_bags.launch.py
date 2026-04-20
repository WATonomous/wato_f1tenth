import os
import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_next_run_number(rosbags_dir):
    os.makedirs(rosbags_dir, exist_ok=True)
    existing = glob.glob(os.path.join(rosbags_dir, 'run_*'))
    if not existing:
        return 1
    numbers = []
    for path in existing:
        basename = os.path.basename(path)
        try:
            numbers.append(int(basename.split('_')[1]))
        except (IndexError, ValueError):
            continue
    return max(numbers, default=0) + 1


def generate_launch_description():
    pure_persuit_config = os.path.join(
        get_package_share_directory('pure_persuit'),
        'config',
        'pure_persuit.yaml'
    )

    pure_persuit_la = DeclareLaunchArgument(
        'pure_persuit_config',
        default_value=pure_persuit_config,
        description='the config for pure persuit settings'
    )

    pure_persuit = Node(
        package='pure_persuit',
        executable='pure_persuit_node',
        name='pure_persuit_node',
        parameters=[LaunchConfiguration('pure_persuit_config')],
        output='screen'
    )

    # --- rosbag recording setup ---
    rosbags_dir = '/home/bolty/ament_ws/src/robot/bringup_robot/rosbags'
    run_number = get_next_run_number(rosbags_dir)
    bag_path = os.path.join(rosbags_dir, f'run_{run_number}')

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-a',               # record all topics
            '-s', 'mcap',       # store as mcap
            '-o', bag_path
        ],
        output='screen'
    )

    # start recording only after pure_persuit launches
    start_bag_after_pp = RegisterEventHandler(
        OnProcessStart(
            target_action=pure_persuit,
            on_start=[rosbag_record]
        )
    )

    ld = LaunchDescription([
        pure_persuit_la,
        pure_persuit,
        start_bag_after_pp
    ])

    return ld