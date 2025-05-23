cmake_minimum_required(VERSION 3.8)
project(gym_vis)

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
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

add_executable(gym_vis_node src/gym_vis.cpp)
target_include_directories(gym_vis_node PUBLIC include)

ament_target_dependencies(gym_vis_node rclcpp std_msgs nav_msgs visualization_msgs sensor_msgs ackermann_msgs tf2_ros tf2_geometry_msgs)


# Install the meshes folder
install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
)

install(TARGETS
  gym_vis_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
