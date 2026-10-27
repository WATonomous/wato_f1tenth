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
find_package(rosidl_default_generators REQUIRED)

# The raceline with its track edges, for planners that need the widths (frenet, MPC)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackWidth.msg"
  "msg/ReferenceTrack.msg"
  DEPENDENCIES std_msgs nav_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)


# Linting runs through pre-commit; unit tests use wato_test (add_wato_test).

add_executable(global_planner_node src/global_planner.cpp)
target_include_directories(global_planner_node PUBLIC include)
target_link_libraries(global_planner_node ${cpp_typesupport_target})
ament_target_dependencies(global_planner_node rclcpp std_msgs nav_msgs geometry_msgs)

install(PROGRAMS scripts/generate_raceline_reference.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
