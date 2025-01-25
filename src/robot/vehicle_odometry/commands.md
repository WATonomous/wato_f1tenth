
In one terminal run:

ros2 run tf2_ros static_transform_publisher \
  0 0 0  0 0 0 1 \
  world f1tenth_1

In other terminal run:

ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
    -p scan_topic:=/autodrive/f1tenth_1/lidar \
    -p use_odometry:=false \
    -p map_frame:=map \
    -p odom_frame:=world \
    -p base_frame:=f1tenth_1

Commands:

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --base-paths src src/robot \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

in ament_ws:
source install/setup.bash

ros2 launch bringup_robot robot.launch.py