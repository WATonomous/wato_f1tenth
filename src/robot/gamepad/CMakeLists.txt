cmake_minimum_required(VERSION 3.8)
project(gamepad)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
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

add_executable(gamepad_node src/gamepad_node.cpp)
target_include_directories(gamepad_node PUBLIC include)

ament_target_dependencies(gamepad_node rclcpp std_msgs sensor_msgs ackermann_msgs)

add_executable(joypad_node src/joypad_teleop.cpp)
target_include_directories(joypad_node PUBLIC include)

ament_target_dependencies(joypad_node rclcpp std_msgs sensor_msgs ackermann_msgs)

install(TARGETS
  gamepad_node
  joypad_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
