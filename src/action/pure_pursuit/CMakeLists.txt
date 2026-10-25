cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# Linting runs through pre-commit; unit tests use wato_test (add_wato_test).

add_executable(pure_pursuit_node src/pure_pursuit.cpp)
target_include_directories(pure_pursuit_node PUBLIC include)
ament_target_dependencies(pure_pursuit_node rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_ros ackermann_msgs visualization_msgs)

install(TARGETS
  pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
