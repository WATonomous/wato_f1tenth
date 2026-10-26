# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.8)
project(sim_util)

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
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# Linting runs through pre-commit; unit tests use wato_test (add_wato_test).

# ackermann converter
add_executable(ackermann_converter_node src/ackermann_converter.cpp)
target_include_directories(ackermann_converter_node PUBLIC include)
ament_target_dependencies(ackermann_converter_node rclcpp std_msgs ackermann_msgs)

# laider scan remapper
add_executable(scan_remap_node src/scan_remap.cpp)
target_include_directories(scan_remap_node PUBLIC include)
ament_target_dependencies(scan_remap_node rclcpp sensor_msgs)

# fake odom
add_executable(fake_odom_node src/fake_odom.cpp)
target_include_directories(fake_odom_node PUBLIC include)
ament_target_dependencies(fake_odom_node rclcpp sensor_msgs std_msgs nav_msgs tf2_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS
  ackermann_converter_node
  scan_remap_node
  fake_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
