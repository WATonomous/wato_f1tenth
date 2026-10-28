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

#include "global_planner.hpp"  // NOLINT(build/include_subdir)

#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>

GlobalPlanner::GlobalPlanner()
: Node("global_planner_node")
{
  // race_tracks/tracks/<TRACK>/raceline.csv, set by world_modeling.launch.yaml
  declare_parameter<std::string>("raceline_file", "");
  declare_parameter<std::string>("path_topic", "/global_planner/path");
  declare_parameter<std::string>("reference_track_topic", "/global_planner/reference_track");
  declare_parameter<std::string>("waypoint_frame_id", "map");

  waypoint_frame_id_ = get_parameter("waypoint_frame_id").as_string();
  const auto qos = rclcpp::QoS(1).transient_local().reliable();
  path_pub_ = create_publisher<nav_msgs::msg::Path>(get_parameter("path_topic").as_string(), qos);
  reference_pub_ =
    create_publisher<global_planner::msg::ReferenceTrack>(get_parameter("reference_track_topic").as_string(), qos);

  const auto file_path = get_parameter("raceline_file").as_string();
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_FATAL(get_logger(), "Could not open raceline reference: '%s' (raceline_file)", file_path.c_str());
    rclcpp::shutdown();
    return;
  }

  reference_track_.header.frame_id = waypoint_frame_id_;
  reference_track_.header.stamp = now();
  reference_track_.path.header = reference_track_.header;
  retrieveData(file);

  RCLCPP_INFO(
    get_logger(),
    "Loaded %zu raceline waypoints with widths from %s",
    reference_track_.path.poses.size(),
    file_path.c_str());
  path_pub_->publish(reference_track_.path);
  reference_pub_->publish(reference_track_);
}

void GlobalPlanner::retrieveData(std::ifstream & file)
{
  std::string line;
  std::getline(file, line);  // x_m,y_m,v_mps,dr_m,dl_m
  while (std::getline(file, line)) {
    std::stringstream row(line);
    std::string x, y, velocity, right, left;
    std::getline(row, x, ',');
    std::getline(row, y, ',');
    std::getline(row, velocity, ',');
    std::getline(row, right, ',');
    std::getline(row, left, ',');

    geometry_msgs::msg::PoseStamped waypoint;
    global_planner::msg::TrackWidth width;
    try {
      waypoint.pose.position.x = std::stod(x);
      waypoint.pose.position.y = std::stod(y);
      waypoint.pose.position.z = std::stod(velocity);
      width.right_m = std::stof(right);
      width.left_m = std::stof(left);
    } catch (const std::exception &) {
      RCLCPP_WARN(get_logger(), "Skipping a line that is not x_m,y_m,v_mps,dr_m,dl_m: %s", line.c_str());
      continue;
    }
    waypoint.header = reference_track_.header;
    reference_track_.path.poses.push_back(waypoint);
    reference_track_.widths.push_back(width);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}
