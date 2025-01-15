#!/bin/bash
set -e

# setup ROS2 environment
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
exec "$@"
