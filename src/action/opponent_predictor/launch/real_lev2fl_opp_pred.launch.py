"""Hardware opponent perception launch for the lev2fl raceline.

Reuses the existing hardware YAMLs (`config/hardware/{lidar_detector,params}.yaml`)
and overrides only `waypoint_path` to point at the lev2fl raceline. Same
detector + predictor binaries as `real_h1_opp_pred.launch.py` — only the
raceline used for projection / scoring differs.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LEV2FL_WAYPOINT_PATH = 'waypoints/old/lev2fl/lev2_1.csv'


def generate_launch_description():
    predictor_params_file = LaunchConfiguration('predictor_params_file')
    detector_params_file = LaunchConfiguration('detector_params_file')
    waypoint_path = LaunchConfiguration('waypoint_path')

    # Override `waypoint_path` for both nodes via the second parameters
    # entry. ROS 2 merges parameter dicts in order — the dict here wins
    # over the YAML's hardcoded houston waypoints.
    waypoint_override = {'waypoint_path': waypoint_path,
                         'waypoint_path_absolute': False}

    detector = Node(
        package='opponent_predictor',
        executable='opponent_lidar_detector_node',
        name='opponent_lidar_detector',
        output='screen',
        parameters=[detector_params_file, waypoint_override],
    )

    predictor = Node(
        package='opponent_predictor',
        executable='opponent_predictor_node',
        name='opponent_predictor',
        output='screen',
        parameters=[predictor_params_file, waypoint_override],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'predictor_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('opponent_predictor'),
                'config', 'hardware',
                'params.yaml',
            ]),
            description='YAML file with opponent predictor parameters.',
        ),
        DeclareLaunchArgument(
            'detector_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('opponent_predictor'),
                'config', 'hardware',
                'lidar_detector.yaml',
            ]),
            description='YAML file with LiDAR opponent detector parameters.',
        ),
        DeclareLaunchArgument(
            'waypoint_path',
            default_value=LEV2FL_WAYPOINT_PATH,
            description='Raceline CSV (relative to mppi_bringup share) used '
                        'by detector + predictor for projection / scoring.',
        ),
        detector,
        predictor,
    ])
