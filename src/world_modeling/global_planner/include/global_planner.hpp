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

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

// message types
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

// #include "visualization_msgs/msg/marker_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

class GlobalPlanner : public rclcpp::Node
{
public:
  GlobalPlanner();

private:
  // publishers
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub;

  // helper functions
  void publish_data();
  void retrieve_data(std::ifstream & file);

  // helper variabels
  nav_msgs::msg::Path waypoints;
  // visualization_msgs::msg::Marker vis_path;

  // parameters
  std::string path_pub_topic, vis_pub_topic, file_directory;
  std::string waypoint_frame_id;
};

#endif
