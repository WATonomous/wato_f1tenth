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
project(wato_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Messages shared across modules. Every image that uses them copies src/wato_msgs in.
# ReferenceTrack: the raceline with its track edges, for planners that need the widths (frenet, MPC)
# PlannerDecision: frenet_planner's per-cycle decision, so Foxglove and ros2 bag can decode it
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackWidth.msg"
  "msg/ReferenceTrack.msg"
  "msg/PlannerDecision.msg"
  DEPENDENCIES std_msgs nav_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
