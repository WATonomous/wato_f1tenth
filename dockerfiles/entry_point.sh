#!/bin/bash
set -e

#source ros2
tail -f /dev/null
source /opt/ros/humble/setup.bash
exec "$@"