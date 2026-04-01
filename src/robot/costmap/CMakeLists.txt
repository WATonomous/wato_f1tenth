cmake_minimum_required(VERSION 3.8)
project(costmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(costmap_node src/costmap_node.cpp)
target_include_directories(costmap_node PUBLIC include)

ament_target_dependencies(costmap_node rclcpp nav_msgs sensor_msgs tf2_ros tf2_geometry_msgs)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS
  costmap_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
