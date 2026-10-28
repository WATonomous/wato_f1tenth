cmake_minimum_required(VERSION 3.5)
project(vesc_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(vesc_msgs
  "msg/VescState.msg"
  "msg/VescStateStamped.msg"
  "msg/VescImu.msg"
  "msg/VescImuStamped.msg"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

# Linting runs through pre-commit; unit tests use wato_test (add_wato_test).

ament_auto_package()
