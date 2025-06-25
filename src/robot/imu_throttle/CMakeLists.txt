cmake_minimum_required(VERSION 3.8)
project(imu_throttle)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(imu_throttle ./imu_throttle.cpp)
target_include_directories(imu_throttle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(imu_throttle rclcpp sensor_msgs geometry_msgs)

install(TARGETS imu_throttle
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(DIRECTORY
  include/
  DESTINATION include/
  OPTIONAL)

ament_package()