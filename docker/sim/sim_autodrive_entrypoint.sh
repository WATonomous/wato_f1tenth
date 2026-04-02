#!/bin/bash
set -e

# Launch API node
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build
ros2 launch autodrive_roboracer bringup_headless.launch.py
