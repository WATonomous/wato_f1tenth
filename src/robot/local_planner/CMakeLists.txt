cmake_minimum_required(VERSION 3.8)
project(local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
endif()

# Pure C++ planning core (no rclcpp dependency).
add_library(local_planner_core STATIC
  src/collision_checker.cpp
  src/rrt_planner.cpp
  src/local_planner_core.cpp
)
target_include_directories(local_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(local_planner_core
  geometry_msgs
  visualization_msgs
)
set_property(TARGET local_planner_core PROPERTY POSITION_INDEPENDENT_CODE ON)

if(BUILD_TESTING)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_rrt_planner.cpp)
    ament_add_gtest(test_rrt_planner test/test_rrt_planner.cpp)
    if(TARGET test_rrt_planner)
      target_link_libraries(test_rrt_planner local_planner_core)
      ament_target_dependencies(test_rrt_planner geometry_msgs)
    endif()
  endif()

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Node executable.
add_executable(local_planner_node src/local_planner_node.cpp)
target_link_libraries(local_planner_node local_planner_core)
ament_target_dependencies(local_planner_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  ament_index_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
