#!/bin/bash
set -e

# Launch API node
source /opt/ros/humble/setup.bash
source install/setup.bash

ls
ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py