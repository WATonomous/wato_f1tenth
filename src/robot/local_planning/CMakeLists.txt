cmake_minimum_required(VERSION 3.8)
project(local_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlanPath.action"
  "msg/PlannerDiagnostics.msg"
  DEPENDENCIES nav_msgs std_msgs
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
)
target_include_directories(state_manager_node PUBLIC include)
ament_target_dependencies(state_manager_node rclcpp rclcpp_action nav_msgs std_msgs geometry_msgs visualization_msgs tf2 tf2_geometry_msgs)
rosidl_get_typesupport_target(state_manager_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(state_manager_node ${state_manager_typesupport})

# local Frenet lattice planner node. The executable name is kept for launch compatibility.
add_executable(hybrid_astar_planner_node
  src/planner/planner_node.cpp
  src/planner/local_frenet_lattice_planner.cpp
  src/planner/quintic_polynomial.cpp
  src/planner/edge_evaluator.cpp
  src/planner/collision_checker.cpp
  src/planner/planner_costs.cpp
  src/frenet_converter.cpp
)
target_include_directories(hybrid_astar_planner_node PUBLIC include)
ament_target_dependencies(hybrid_astar_planner_node
  rclcpp
  rclcpp_action
  nav_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)
rosidl_get_typesupport_target(planner_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(hybrid_astar_planner_node ${planner_typesupport})

add_executable(occupancy_grid_frame_adapter_node src/occupancy_grid_frame_adapter.cpp)
target_include_directories(occupancy_grid_frame_adapter_node PUBLIC include)
ament_target_dependencies(occupancy_grid_frame_adapter_node rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  state_manager_node
  hybrid_astar_planner_node
  occupancy_grid_frame_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
