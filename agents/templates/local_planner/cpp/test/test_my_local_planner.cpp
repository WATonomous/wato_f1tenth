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

// Unit tests for the planner (no ROS graph needed)

#include <catch2/catch_all.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "my_local_planner/my_local_planner.hpp"

using my_local_planner::plan;

TEST_CASE("an empty track keeps the global raceline or plans ahead", "[my_local_planner]")
{
  my_local_planner::Params params;
  my_local_planner::State state;
  sensor_msgs::msg::LaserScan scan;  // no obstacles
  nav_msgs::msg::Path global_path;
  global_path.header.frame_id = "map";
  for (int i = 0; i < 50; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = 0.1 * i;
    p.pose.position.z = 1.0;
    global_path.poses.push_back(p);
  }

  const auto path = plan(state, scan, global_path, params);
  if (path) {
    // a planned path starts at the car (base_link origin)
    REQUIRE_FALSE(path->poses.empty());
    REQUIRE(path->poses.front().pose.position.x < 0.5);
  }
}

// TODO(you): once plan() is written, test its behaviour, for example an obstacle 2 m ahead on the
// raceline gives a path that passes it on one side.
