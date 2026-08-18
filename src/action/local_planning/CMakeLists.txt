cmake_minimum_required(VERSION 3.8)
project(local_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Without this a clean colcon build defaults to no optimisation flags at all,
# and every candidate-generation timing measured from it is meaningless.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(global_planner REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/PlannerDecision.msg"
  DEPENDENCIES builtin_interfaces std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ---------------------------------------------------------------------------
# local_planning_core
#
# Geometry, curves, maneuvers, collision, speed, selection.  Deliberately links
# nothing from ROS: no rclcpp, no subscriptions, no declare_parameter reached
# three layers down, no now(), no TF.  Config arrives as a plain struct and
# time arrives as an argument.  Keeping that true costs nothing and is what
# leaves a one-off debug binary possible if one ever becomes worth writing.
# ---------------------------------------------------------------------------
add_library(local_planning_core
  src/core/types.cpp
  src/reference/raceline_reference.cpp
  src/reference/reference_window.cpp
  src/curves/frenet_polynomial.cpp
  src/curves/frenet_connection_generator.cpp
  src/maneuvers/maneuver_builder.cpp
  src/collision/collision_checker.cpp
  src/collision/track_bounds_checker.cpp
  src/speed/velocity_profile.cpp
  src/state/racing_state_machine.cpp
  src/selection/candidate_selector.cpp
  src/planning/braking_path_generator.cpp
  src/planning/local_planner.cpp
)
target_include_directories(local_planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(local_planning_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/third_party/thread_pool
)
# PRIVATE, not PUBLIC: the worker pool is an implementation detail behind a
# private header (src/core/worker_pool.hpp), so Threads has no business in the
# exported link interface.  As PUBLIC it landed in export_local_planningExport
# without a matching find_package, and every downstream consumer -- costmap_test_tools
# links this for RacelineReference -- failed to configure.
target_link_libraries(local_planning_core PRIVATE Threads::Threads)

# ---------------------------------------------------------------------------
# local_planning_ros — the boundary layer plus the nodes.
# ---------------------------------------------------------------------------
add_library(local_planning_ros
  src/ros/planner_diagnostics.cpp
  src/ros/planner_visualization.cpp
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
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(occupancy_grid_frame_adapter_node src/ros/occupancy_grid_frame_adapter_node.cpp)
ament_target_dependencies(occupancy_grid_frame_adapter_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(planner_node src/ros/planner_node.cpp)
target_link_libraries(planner_node
  local_planning_ros
  local_planning_core
  ${cpp_typesupport_target}
)
ament_target_dependencies(planner_node
  rclcpp ackermann_msgs nav_msgs std_msgs geometry_msgs visualization_msgs global_planner
  tf2 tf2_ros tf2_geometry_msgs)

# Only the core is exported; costmap_test_tools links it for RacelineReference.
install(DIRECTORY include/ DESTINATION include)

install(TARGETS local_planning_core
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

install(TARGETS local_planning_ros
  DESTINATION lib
)

install(TARGETS
  occupancy_grid_frame_adapter_node
  planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # colcon is run from inside this package, so build/, install/ and log/ land in
  # the source tree and every linter walks into them: the rosidl-generated
  # headers and the Python message bindings get linted as if they were ours, and
  # the ~200 failures they raise bury the handful that are real.  AMENT_IGNORE is
  # the one exclusion mechanism all of them honour -- AMENT_LINT_AUTO_FILE_EXCLUDE
  # is matched against exact file paths by uncrustify and lint_cmake, so naming
  # the directories there does nothing.  Written from here rather than committed
  # so that wiping the generated trees restores the markers on the next configure.
  foreach(_generated_dir build install log)
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/${_generated_dir}/AMENT_IGNORE "")
  endforeach()
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_raceline_reference test/test_raceline_reference.cpp)
  target_link_libraries(test_raceline_reference local_planning_core)

  ament_add_gtest(test_frenet_connection_generator test/test_frenet_connection_generator.cpp)
  target_link_libraries(test_frenet_connection_generator local_planning_core)

  ament_add_gtest(test_maneuver_candidate_builder test/test_maneuver_candidate_builder.cpp)
  target_link_libraries(test_maneuver_candidate_builder local_planning_core)

  ament_add_gtest(test_collision_checker test/test_collision_checker.cpp)
  target_link_libraries(test_collision_checker local_planning_core)

  ament_add_gtest(test_track_bounds_checker test/test_track_bounds_checker.cpp)
  target_link_libraries(test_track_bounds_checker local_planning_core)

  ament_add_gtest(test_velocity_profile test/test_velocity_profile.cpp)
  target_link_libraries(test_velocity_profile local_planning_core)

  ament_add_gtest(test_racing_state_machine test/test_racing_state_machine.cpp)
  target_link_libraries(test_racing_state_machine local_planning_core)

  ament_add_gtest(test_candidate_selector test/test_candidate_selector.cpp)
  target_link_libraries(test_candidate_selector local_planning_core)

  ament_add_gtest(test_braking_path_generator test/test_braking_path_generator.cpp)
  target_link_libraries(test_braking_path_generator local_planning_core)

  ament_add_gtest(test_local_planner test/test_local_planner.cpp)
  target_link_libraries(test_local_planner local_planning_core)

  ament_add_gtest(test_ros_adapters test/test_ros_adapters.cpp)
  target_link_libraries(test_ros_adapters local_planning_ros)
  ament_target_dependencies(test_ros_adapters rclcpp)

  ament_add_gtest(test_planner_diagnostics test/test_planner_diagnostics.cpp)
  target_link_libraries(test_planner_diagnostics local_planning_ros)
  ament_target_dependencies(test_planner_diagnostics rclcpp)

  ament_add_gtest(test_planner_visualization test/test_planner_visualization.cpp)
  target_link_libraries(test_planner_visualization local_planning_ros)
  ament_target_dependencies(test_planner_visualization rclcpp visualization_msgs)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
