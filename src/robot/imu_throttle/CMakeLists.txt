cmake_minimum_required(VERSION 3.8)
project(imu_throttle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(imu_throttle src/imu_throttle.cpp)
target_include_directories(imu_throttle PUBLIC include)
ament_target_dependencies(imu_throttle
  rclcpp
  std_msgs
  sensor_msgs
  ackermann_msgs)


# Install the meshes folder
install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
)

install(TARGETS
  imu_throttle
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
