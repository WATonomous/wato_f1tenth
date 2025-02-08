#!/bin/bash
set -e

# Launch API node
source /opt/ros/humble/setup.bash
source install/setup.bash

sed -i '284c\        autodrive.lidar_range_array = np.fromstring(data["V1 LIDAR Range Array"], sep="\\n")' src/autodrive_devkit/autodrive_f1tenth/autodrive_bridge.py
colcon build
ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py