cmake_minimum_required(VERSION 3.8)
project(odom_listener_pkg)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(odom_listener_node src/odom_listener_node.cpp)
ament_target_dependencies(odom_listener_node rclcpp nav_msgs)

install(TARGETS odom_listener_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()  
