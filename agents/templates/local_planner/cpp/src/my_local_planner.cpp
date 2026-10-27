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

#include "my_local_planner/my_local_planner.hpp"

namespace my_local_planner
{

std::optional<nav_msgs::msg::Path> plan(
  const State & state,
  const sensor_msgs::msg::LaserScan & scan,
  const nav_msgs::msg::Path & global_path,
  const Params & params)
{
  (void)state;
  (void)scan;
  (void)global_path;
  (void)params;

  // TODO(you): your planner. Until then the controller keeps following the global raceline.
  return std::nullopt;
}

}  // namespace my_local_planner
