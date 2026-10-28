// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// The planner: everything you write lives in plan() (src/my_local_planner.cpp).
// my_local_planner_node handles topics, timing, stale inputs and the active flag.

#pragma once

#include <optional>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace my_local_planner
{

// Tunables, set from config/{car,sim}/my_local_planner.yaml. Add yours here and in
// MyLocalPlannerNode's constructor.
struct Params
{
  double horizon_m = 3.0;  // how far ahead to plan
};

// The car, in the map frame
struct State
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;  // rad
  double speed = 0.0;  // m/s, from wheel odometry
};

// Returns the path the controller should follow now, in base_link (x forward, y left), starting
// at the car, with each pose's position.z the target speed there. Return std::nullopt when the
// global raceline is fine as it is: the controller then follows /global_planner/path.
// global_path is the raceline in the map frame (a closed loop); scan is in its own frame (laser).
std::optional<nav_msgs::msg::Path> plan(
  const State & state,
  const sensor_msgs::msg::LaserScan & scan,
  const nav_msgs::msg::Path & global_path,
  const Params & params);

}  // namespace my_local_planner
