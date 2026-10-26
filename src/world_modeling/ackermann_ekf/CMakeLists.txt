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
project(ackermann_ekf)

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
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)


# Linting runs through pre-commit; unit tests use wato_test (add_wato_test).

#ekf node
add_executable(ekf_node src/ekf_node.cpp)
target_include_directories(ekf_node PUBLIC include)
ament_target_dependencies(ekf_node rclcpp std_msgs nav_msgs sensor_msgs tf2_ros tf2_geometry_msgs)

#ekf node
add_executable(ekf src/ekf.cpp)
target_include_directories(ekf PUBLIC include)
ament_target_dependencies(ekf rclcpp nav_msgs sensor_msgs tf2_ros tf2_geometry_msgs ackermann_msgs)

install(
  TARGETS ekf_node ekf
  DESTINATION lib/${PROJECT_NAME})

ament_package()
