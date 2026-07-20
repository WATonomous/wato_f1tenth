cmake_minimum_required(VERSION 3.8)
project(local_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlannerIntent.msg"
  "msg/PlannerStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# state manager node
add_executable(state_manager_node
  src/state_manager/state_manager_node.cpp
  src/state_manager/state_manager_code.cpp
  src/frenet_converter.cpp
  src/ros_adapters.cpp
)
target_include_directories(state_manager_node PUBLIC include)
ament_target_dependencies(state_manager_node rclcpp nav_msgs std_msgs geometry_msgs tf2 tf2_geometry_msgs)
rosidl_get_typesupport_target(state_manager_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(state_manager_node ${state_manager_typesupport})

set(LOCAL_FRENET_LATTICE_PLANNER_SOURCES
  src/planner/planner_node.cpp
  src/planner/local_frenet_lattice_planner.cpp
  src/planner/frenet_polynomial.cpp
  src/planner/edge_evaluator.cpp
  src/planner/collision_checker.cpp
  src/planner/planner_costs.cpp
  src/planner/path_processing.cpp
  src/planner/spline_refinement.cpp
  src/frenet_converter.cpp
  src/ros_adapters.cpp
)

set(LOCAL_FRENET_LATTICE_PLANNER_DEPS
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  ackermann_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# local Frenet lattice planner node.
add_executable(local_frenet_lattice_planner_node
  ${LOCAL_FRENET_LATTICE_PLANNER_SOURCES}
)
target_include_directories(local_frenet_lattice_planner_node PUBLIC include)
ament_target_dependencies(local_frenet_lattice_planner_node
  ${LOCAL_FRENET_LATTICE_PLANNER_DEPS}
)
rosidl_get_typesupport_target(planner_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(local_frenet_lattice_planner_node ${planner_typesupport})

# TODO: remove after launch files and operator scripts stop using the old Hybrid A* name.
add_executable(hybrid_astar_planner_node
  ${LOCAL_FRENET_LATTICE_PLANNER_SOURCES}
)
target_include_directories(hybrid_astar_planner_node PUBLIC include)
ament_target_dependencies(hybrid_astar_planner_node
  ${LOCAL_FRENET_LATTICE_PLANNER_DEPS}
)
target_link_libraries(hybrid_astar_planner_node ${planner_typesupport})

add_executable(occupancy_grid_frame_adapter_node src/occupancy_grid_frame_adapter.cpp)
target_include_directories(occupancy_grid_frame_adapter_node PUBLIC include)
ament_target_dependencies(occupancy_grid_frame_adapter_node rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(racing_line_publisher_node src/racing_line_publisher_node.cpp)
ament_target_dependencies(racing_line_publisher_node rclcpp nav_msgs std_msgs geometry_msgs)

add_executable(bool_topic_publisher_node src/racing_line_publisher_node.cpp)
ament_target_dependencies(bool_topic_publisher_node rclcpp nav_msgs std_msgs geometry_msgs)

install(TARGETS
  state_manager_node
  local_frenet_lattice_planner_node
  hybrid_astar_planner_node
  occupancy_grid_frame_adapter_node
  racing_line_publisher_node
  bool_topic_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
