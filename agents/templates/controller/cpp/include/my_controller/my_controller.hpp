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

// The control law: everything you write lives in compute() (src/my_controller.cpp).
// my_controller_node handles topics, the dead man switch, path choice and the watchdog.

#pragma once

#include <optional>

#include <nav_msgs/msg/path.hpp>

namespace my_controller
{

// Tunables, set from config/{car,sim}/my_controller.yaml. Add your gains here and in
// MyControllerNode's constructor.
struct Params
{
  double max_speed = 1.0;  // m/s
  double max_steering_angle = 0.4;  // rad
  double wheelbase = 0.33;  // m
};

// The car, in the map frame
struct State
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;  // rad
  double speed = 0.0;  // m/s, from wheel odometry
};

struct Command
{
  double steering_angle = 0.0;  // rad, positive = left
  double speed = 0.0;  // m/s
  std::optional<double> debug_x;  // optional point to show in Foxglove (path frame)
  std::optional<double> debug_y;
};

// path is either
//   - the global raceline: map frame, a closed loop (path_is_local = false), or
//   - the local planner's path: base_link frame, open, starting at the car (path_is_local = true).
// Each pose's position.z is the target speed there. The node clamps the result to the limits.
Command compute(const State & state, const nav_msgs::msg::Path & path, bool path_is_local, const Params & params);

}  // namespace my_controller
