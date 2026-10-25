cmake_minimum_required(VERSION 3.8)
project(global_planner)

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
find_package(geometry_msgs REQUIRED)
#find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


# Linting runs through pre-commit; unit tests use wato_test (add_wato_test).

add_executable(global_planner_node src/global_planner.cpp)
target_include_directories(global_planner_node PUBLIC include)
ament_target_dependencies(global_planner_node rclcpp std_msgs nav_msgs geometry_msgs ament_index_cpp)

install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
)

install(TARGETS
  global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
