cmake_minimum_required(VERSION 3.8)
project(opponent_predictor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(opponent_predictor_node src/opponent_predictor_node.cpp)
ament_target_dependencies(opponent_predictor_node
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_executable(opponent_lidar_detector_node src/opponent_lidar_detector_node.cpp)
ament_target_dependencies(opponent_lidar_detector_node
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

install(
  TARGETS opponent_predictor_node opponent_lidar_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
