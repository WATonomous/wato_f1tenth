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

// Unit tests for the control law (no ROS graph needed)

#include <cmath>

#include <catch2/catch_all.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "my_controller/my_controller.hpp"

using my_controller::compute;

namespace
{

// A straight raceline along +x at 1 m/s
nav_msgs::msg::Path straight_path()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i < 50; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = 0.1 * i;
    p.pose.position.z = 1.0;
    path.poses.push_back(p);
  }
  return path;
}

}  // namespace

TEST_CASE("compute stays within the limits", "[my_controller]")
{
  my_controller::Params params;
  my_controller::State state;
  const auto cmd = compute(state, straight_path(), false, params);
  REQUIRE(std::abs(cmd.steering_angle) <= params.max_steering_angle);
  REQUIRE(cmd.speed >= 0.0);
  REQUIRE(cmd.speed <= params.max_speed);
}

// TODO(you): once compute() is written, test its behaviour, for example:
//   - on the line and facing along it: steering_angle is about 0
//   - offset to the right of the line (state.y = -0.5): steering_angle > 0 (turns left)
