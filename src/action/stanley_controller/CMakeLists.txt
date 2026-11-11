cmake_minimum_required(VERSION 3.8)
project(stanley_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Linting runs through pre-commit; unit tests use wato_test (add_wato_test).

# The control law, without ROS plumbing, so unit tests can call it directly
add_library(stanley_controller_lib src/stanley.cpp)
target_include_directories(stanley_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(stanley_controller_lib nav_msgs geometry_msgs)

add_executable(stanley_controller src/stanley_controller.cpp)
target_include_directories(stanley_controller PUBLIC include)
target_link_libraries(stanley_controller stanley_controller_lib)
ament_target_dependencies(stanley_controller rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs ackermann_msgs tf2 tf2_ros tf2_geometry_msgs
  visualization_msgs)

install(TARGETS stanley_controller_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS stanley_controller
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(wato_test REQUIRED)
  add_wato_test(test_stanley
    test/test_stanley.cpp
    LIBRARIES
      stanley_controller_lib
  )
endif()

ament_package()
