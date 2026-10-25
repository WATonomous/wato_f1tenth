"""Hardware opponent perception launch for the e7_fifth_v3 track.

Runs the detector + predictor with e7-specific YAMLs
(`config/hardware/e7_lidar_detector.yaml`, `config/hardware/e7_params.yaml`).
Those YAMLs already point the opponent pipeline at the *exact same* raceline
MPPI follows (`mppi_bringup/waypoints/e7.csv`, loaded by
`mppi_bringup/launch/e7.launch.py`) with `reverse_waypoints: false`, so there
are NO parameter overrides here -- every value lives in the YAML.

Prerequisites (started separately):
  - jetson_pf.launch.py -> /scan, /pf/pose/odom, and /map (e7_fifth_v3)
  - e7.launch.py         -> MPPI controller (consumes /opponent/predicted_path)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    predictor_params_file = LaunchConfiguration('predictor_params_file')
    detector_params_file = LaunchConfiguration('detector_params_file')

    detector = Node(
        package='opponent_predictor',
        executable='opponent_lidar_detector_node',
        name='opponent_lidar_detector',
        output='screen',
        parameters=[detector_params_file],
    )

    predictor = Node(
        package='opponent_predictor',
        executable='opponent_predictor_node',
        name='opponent_predictor',
        output='screen',
        parameters=[predictor_params_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'predictor_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('opponent_predictor'),
                'config', 'hardware',
                'e7_params.yaml',
            ]),
            description='YAML file with opponent predictor parameters (e7).',
        ),
        DeclareLaunchArgument(
            'detector_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('opponent_predictor'),
                'config', 'hardware',
                'e7_lidar_detector.yaml',
            ]),
            description='YAML file with LiDAR opponent detector parameters (e7).',
        ),
        detector,
        predictor,
    ])
