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

#ifndef FRENET_PLANNER_ROS_ROS_ADAPTERS_HPP
#define FRENET_PLANNER_ROS_ROS_ADAPTERS_HPP

#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/time.hpp>

#include "frenet_planner/core/types.hpp"
#include "frenet_planner/planning/local_planner.hpp"
#include "wato_msgs/msg/planner_decision.hpp"

namespace frenet_planner
{

// ROS ↔ core type boundary.
Odometry rosToOdometry(const nav_msgs::msg::Odometry & msg);
OccupancyGrid rosToOccupancyGrid(const nav_msgs::msg::OccupancyGrid & msg);
nav_msgs::msg::Path pathToRos(const Path & path, const rclcpp::Time & stamp, const std::string & frame_id);
wato_msgs::msg::PlannerDecision plannerDecisionToRos(
  const PlannerDecisionData & data, const rclcpp::Time & stamp, const std::string & frame_id);

}  // namespace frenet_planner

#endif  // FRENET_PLANNER_ROS_ROS_ADAPTERS_HPP
