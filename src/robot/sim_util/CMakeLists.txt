cmake_minimum_required(VERSION 3.8)
project(sim_util)

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
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

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

# ackermann converter
add_executable(ackermann_converter_node src/ackermann_converter.cpp)
target_include_directories(ackermann_converter_node PUBLIC include)
ament_target_dependencies(ackermann_converter_node rclcpp std_msgs ackermann_msgs)

# laider scan remapper 
add_executable(scan_remap_node src/scan_remap.cpp)
target_include_directories(scan_remap_node PUBLIC include)
ament_target_dependencies(scan_remap_node rclcpp sensor_msgs)

# fake odom
add_executable(fake_odom_node src/fake_odom.cpp)
target_include_directories(fake_odom_node PUBLIC include)
ament_target_dependencies(fake_odom_node rclcpp sensor_msgs std_msgs nav_msgs tf2_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS
  ackermann_converter_node
  scan_remap_node
  fake_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
