from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()  # Begin building a launch description

    # Dynamically find the path to the package
    pkg_ekf = get_package_share_directory('bringup_robot')

    # Path to the bag directory inside the package (2 files options : 1.ekf_testdata_steady_lap 2.ekf_testdata_fast_lap )
    bag_path = os.path.join(pkg_ekf, 'bags', 'ekf_testdata_fast_lap')

    # Play the bag file
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path],
        output='screen'
    )
    ld.add_action(bag_play)

    # EKF node
    ekf_node = Node(
        package="ackermann_ekf",
        executable="ekf_node",
        name="ekf_node",
        output="screen"
    )
    ld.add_action(ekf_node)

    # Shutdown everything when the bag finishes
    shutdown_on_bag_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=bag_play,
            on_exit=[Shutdown()]
        )
    )
    ld.add_action(shutdown_on_bag_exit)

    return ld

