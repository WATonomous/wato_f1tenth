# Hardware launch for e7_fifth_v3 (Jetson Orin). Localization
# (particle_filter, map_server) is brought up separately by
# bringup_robot/launch/jetson_pf.launch.py -- this file only starts the MPPI
# controller, in place of bringup_robot/launch/pure_persuit.launch.py.
#
# mppi_node hardcodes its drive publisher to "/drive", which would bypass
# bringup_robot's ackermann_mux safety arbitration (bringup_robot/config/mux.yaml:
# /drive/ebreak, /drive/joystick, /drive/autonomy -> muxed into ackermann_cmd).
# The remapping below routes MPPI's output into the "autonomy" mux input,
# same as pure_persuit_node, so e-stop/joystick priority still works.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    drive_topic = LaunchConfiguration('drive_topic')
    wall_map_yaml = LaunchConfiguration('wall_map_yaml')
    wpt_path = LaunchConfiguration('wpt_path')

    # Stop drive while MPPI warms up (JAX compile on first control_step can
    # take a moment)
    stop_drive = ExecuteProcess(
        cmd=[
            'ros2',
            'topic',
            'pub',
            '--times',
            '10',
            '--rate',
            '10',
            '--print',
            '0',
            '--wait-matching-subscriptions',
            '0',
            drive_topic,
            'ackermann_msgs/msg/AckermannDriveStamped',
            '{drive: {steering_angle: 0.0, speed: 0.0}}',
        ],
        output='screen',
    )

    mppi_node = Node(
        package='mppi_example',
        executable='mppi_node',
        name='lmppi_node',
        output='screen',
        parameters=[params_file, {
            'wpt_path': wpt_path,
            'wpt_path_absolute': True,
            'wall_cost_map_yaml': wall_map_yaml,
        }],
        remappings=[
            ('/drive', drive_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mppi_bringup'),
                'config',
                'params_e7.yaml',
            ]),
            description='YAML with MPPI ROS2 params for e7_fifth_v3',
        ),
        DeclareLaunchArgument(
            'wpt_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('mppi_bringup'),
                'waypoints',
                'wato_office.csv',
            ]),
            description='Raceline CSV consumed by mppi_node',
        ),
        DeclareLaunchArgument(
            'drive_topic',
            default_value='/drive/autonomy',
            description='Ackermann drive topic MPPI publishes to (bringup_robot ackermann_mux "autonomy" input, not /drive directly)',
        ),
        DeclareLaunchArgument(
            'wall_map_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('bringup_robot'),
                'maps',
                'wato_office.yaml',
            ]),
            description="Static map yaml used to build MPPI's wall-distance cost field",
        ),
        stop_drive,
        TimerAction(
            period=1.6,
            actions=[mppi_node],
        ),
    ])
