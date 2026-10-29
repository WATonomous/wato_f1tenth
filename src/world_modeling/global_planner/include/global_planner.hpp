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

#ifndef GLOBAL_PLANNER_HPP_
#define GLOBAL_PLANNER_HPP_

#include <fstream>
#include <memory>
#include <string>

#include "wato_msgs/msg/reference_track.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class GlobalPlanner : public rclcpp::Node
{
public:
  GlobalPlanner();

private:
  void retrieveData(std::ifstream & file);

  wato_msgs::msg::ReferenceTrack reference_track_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<wato_msgs::msg::ReferenceTrack>::SharedPtr reference_pub_;
  std::string waypoint_frame_id_;
};

#endif  // GLOBAL_PLANNER_HPP_
