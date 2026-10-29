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

#ifndef FRENET_PLANNER_ROS_PLANNER_VISUALIZATION_HPP
#define FRENET_PLANNER_ROS_PLANNER_VISUALIZATION_HPP

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "frenet_planner/planning/local_planner.hpp"
#include "frenet_planner/reference/raceline_reference.hpp"
#include "frenet_planner/state/racing_state_machine.hpp"

namespace frenet_planner
{

struct PlannerVisualizationConfig
{
  std::string map_frame;
  bool publish_projection_markers = true;
  double vehicle_full_width_m = 0.0;
  double compat_heading_rad = 0.0;
};

struct CandidateFamily  // pre-filter candidates from ManeuverBuilder
{
  CandidateSource source = CandidateSource::NONE;
  std::vector<ManeuverCandidate> candidates;
};

class PlannerVisualization
{
public:
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using MarkerPublisher = rclcpp::Publisher<MarkerArray>;

  PlannerVisualization(const RacelineReference & reference, PlannerVisualizationConfig config);

  void publishCandidates(const LocalPlanResult & result, const rclcpp::Time & stamp, MarkerPublisher & publisher) const;
  void publishAllCandidates(
    const std::vector<CandidateFamily> & families, const rclcpp::Time & stamp, MarkerPublisher & publisher) const;
  void publishProjection(
    const Odometry & odom, const TacticalState & state, const rclcpp::Time & stamp, MarkerPublisher & publisher) const;
  void publishTrackBounds(const rclcpp::Time & stamp, MarkerPublisher & publisher) const;

private:
  void appendBrakingDetail(
    const LocalPlanResult & result, const std_msgs::msg::Header & header, MarkerArray & markers) const;

  const RacelineReference & reference_;
  const PlannerVisualizationConfig config_;
};

}  // namespace frenet_planner

#endif  // FRENET_PLANNER_ROS_PLANNER_VISUALIZATION_HPP
