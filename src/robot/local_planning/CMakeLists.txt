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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlanPath.action"
  DEPENDENCIES geometry_msgs nav_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# state manager node
add_executable(state_manager_node src/state_manager_node.cpp src/racing_state_machine.cpp src/frenet_converter.cpp)
target_include_directories(state_manager_node PUBLIC include)
ament_target_dependencies(state_manager_node rclcpp rclcpp_action nav_msgs std_msgs geometry_msgs visualization_msgs)
rosidl_get_typesupport_target(state_manager_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(state_manager_node ${state_manager_typesupport})

# hybrid astar planner node
add_executable(hybrid_astar_planner_node src/hybrid_astar_planner_node.cpp src/hybrid_astar_planner.cpp src/frenet_converter.cpp)
target_include_directories(hybrid_astar_planner_node PUBLIC include)
ament_target_dependencies(hybrid_astar_planner_node rclcpp rclcpp_action nav_msgs std_msgs geometry_msgs visualization_msgs)
rosidl_get_typesupport_target(planner_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(hybrid_astar_planner_node ${planner_typesupport})

install(TARGETS
  state_manager_node
  hybrid_astar_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
