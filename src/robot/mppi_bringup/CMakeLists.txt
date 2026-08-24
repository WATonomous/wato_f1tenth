cmake_minimum_required(VERSION 3.8)
project(mppi_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch waypoints maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
