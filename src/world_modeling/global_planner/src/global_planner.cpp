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

#include <cmath>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

GlobalPlanner::GlobalPlanner()
: Node("global_planner_node")
{
  // race_tracks/tracks/<TRACK>/raceline.csv, set by world_modeling.launch.yaml
  declare_parameter<std::string>("raceline_file", "");
  declare_parameter<std::string>("path_topic", "/global_planner/path");
  declare_parameter<std::string>("reference_track_topic", "/global_planner/reference_track");
  declare_parameter<std::string>("edges_topic", "/global_planner/edges");
  declare_parameter<std::string>("waypoint_frame_id", "map");

  waypoint_frame_id_ = get_parameter("waypoint_frame_id").as_string();
  const auto qos = rclcpp::QoS(1).transient_local().reliable();
  path_pub_ = create_publisher<nav_msgs::msg::Path>(get_parameter("path_topic").as_string(), qos);
  reference_pub_ =
    create_publisher<global_planner::msg::ReferenceTrack>(get_parameter("reference_track_topic").as_string(), qos);
  // Published once, so latched like the path; it is what Foxglove draws the track edges from
  edges_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(get_parameter("edges_topic").as_string(), qos);

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
  edges_pub_->publish(trackEdges());
}

// The left and right track edges as two line strips: each raceline point moved along the normal of
// the local direction of travel by its dl_m (left) and dr_m (right). The raceline is a closed loop.
visualization_msgs::msg::MarkerArray GlobalPlanner::trackEdges() const
{
  visualization_msgs::msg::MarkerArray edges;
  const auto & poses = reference_track_.path.poses;
  const size_t n = poses.size();
  for (int side = 0; side < 2; ++side) {
    visualization_msgs::msg::Marker line;
    line.header = reference_track_.header;
    line.ns = side == 0 ? "left" : "right";
    line.id = side;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.05;
    line.color.r = 1.0;
    line.color.g = 0.6;
    line.color.a = 1.0;
    for (size_t i = 0; i <= n && n > 2; ++i) {
      const auto & here = poses[i % n].pose.position;
      const auto & next = poses[(i + 1) % n].pose.position;
      const auto & prev = poses[(i + n - 1) % n].pose.position;
      const double tx = next.x - prev.x, ty = next.y - prev.y;
      const double norm = std::hypot(tx, ty);
      if (norm < 1e-9) {
        continue;
      }
      const double nx = -ty / norm, ny = tx / norm;  // left of the direction of travel
      const auto & width = reference_track_.widths[i % n];
      const double offset = side == 0 ? width.left_m : -width.right_m;
      geometry_msgs::msg::Point p;
      p.x = here.x + nx * offset;
      p.y = here.y + ny * offset;
      line.points.push_back(p);
    }
    edges.markers.push_back(line);
  }
  return edges;
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
