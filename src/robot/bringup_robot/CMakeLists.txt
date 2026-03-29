cmake_minimum_required(VERSION 3.5)
project(bringup_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch maps config bags
  DESTINATION share/${PROJECT_NAME})

ament_package()
