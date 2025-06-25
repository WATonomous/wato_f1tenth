#!/bin/bash
set -e

source /opt/watonomous/setup.bash
source /opt/ros/humble/setup.bash

cd /home/bolty/ament_ws
colcon build --merge-install --symlink-install
source install/setup.bash

exec ros2 run imu_throttle imu_throttle