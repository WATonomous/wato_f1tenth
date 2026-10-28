# Hardware launch for e7_fifth_v3 (Jetson Orin), CUDA MPPI (mppi_cpp).
# Mirrors mppi_bringup/launch/e7.launch.py: localization (particle_filter,
# map_server) is brought up separately by world_modeling.launch.yaml.
#
# mppi_node publishes to "/drive"; the remapping below routes it into
# the ackermann_mux "autonomy" input (/hal/drive/autonomy) so e-stop/joystick priority
# still works.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    drive_topic = LaunchConfiguration("drive_topic")
    wall_map_yaml = LaunchConfiguration("wall_map_yaml")
    wpt_path = LaunchConfiguration("wpt_path")

    # Stop drive while MPPI warms up (CUDA context init + SDF build)
    stop_drive = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--times",
            "10",
            "--rate",
            "10",
            "--print",
            "0",
            "--wait-matching-subscriptions",
            "0",
            drive_topic,
            "ackermann_msgs/msg/AckermannDriveStamped",
            "{drive: {steering_angle: 0.0, speed: 0.0}}",
        ],
        output="screen",
    )

    mppi_node = Node(
        package="mppi_cpp",
        executable="mppi_node",
        name="lmppi_node",
        output="screen",
        parameters=[
            params_file,
            {
                "wpt_path": wpt_path,
                "wpt_path_absolute": True,
                "wall_cost_map_yaml": wall_map_yaml,
            },
        ],
        remappings=[
            ("/drive", drive_topic),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("mppi_cpp"),
                        "config",
                        "params_e7.yaml",
                    ]
                ),
                description="YAML with MPPI ROS2 params for e7_fifth_v3",
            ),
            DeclareLaunchArgument(
                "wpt_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("race_tracks"),
                        "tracks",
                        "wato_office",
                        "raceline_mppi.csv",
                    ]
                ),
                description="Raceline CSV consumed by mppi_node",
            ),
            DeclareLaunchArgument(
                "drive_topic",
                default_value="/hal/drive/autonomy",
                description='Ackermann drive topic MPPI publishes to (the ackermann_mux "autonomy" input, not /drive directly)',
            ),
            DeclareLaunchArgument(
                "wall_map_yaml",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("race_tracks"),
                        "tracks",
                        "wato_office",
                        "map.yaml",
                    ]
                ),
                description="Static map yaml used to build MPPI's wall-distance cost field",
            ),
            stop_drive,
            TimerAction(period=1.6, actions=[mppi_node]),
        ]
    )
