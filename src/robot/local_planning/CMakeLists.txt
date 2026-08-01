cmake_minimum_required(VERSION 3.8)
project(local_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlannerIntent.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ---------------------------------------------------------------------------
# local_planning_core
#
# Geometry, curves, sampling, collision, speed, selection.  Deliberately links
# nothing from ROS: no rclcpp, no subscriptions, no declare_parameter reached
# three layers down, no now(), no TF.  Config arrives as a plain struct and
# time arrives as an argument.  Keeping that true costs nothing and is what
# leaves a one-off debug binary possible if one ever becomes worth writing.
# ---------------------------------------------------------------------------
add_library(local_planning_core
  src/core/types.cpp
  src/reference/raceline_reference.cpp
  src/collision/collision_checker.cpp
  src/speed/velocity_profile.cpp
  src/state/racing_state_machine.cpp
)
target_include_directories(local_planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# ---------------------------------------------------------------------------
# local_planning_ros — the boundary layer plus the nodes.
# ---------------------------------------------------------------------------
add_library(local_planning_ros
  src/ros/ros_adapters.cpp
)
target_link_libraries(local_planning_ros PUBLIC
  local_planning_core
  ${cpp_typesupport_target}
)
ament_target_dependencies(local_planning_ros PUBLIC
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(state_manager_node src/ros/state_manager_node.cpp)
target_link_libraries(state_manager_node local_planning_ros)

add_executable(occupancy_grid_frame_adapter_node src/ros/occupancy_grid_frame_adapter_node.cpp)
ament_target_dependencies(occupancy_grid_frame_adapter_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(racing_line_publisher_node src/ros/racing_line_publisher_node.cpp)
ament_target_dependencies(racing_line_publisher_node rclcpp nav_msgs geometry_msgs)

install(TARGETS local_planning_core local_planning_ros
  DESTINATION lib
)

install(TARGETS
  state_manager_node
  occupancy_grid_frame_adapter_node
  racing_line_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_raceline_reference test/test_raceline_reference.cpp)
  target_link_libraries(test_raceline_reference local_planning_core)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
