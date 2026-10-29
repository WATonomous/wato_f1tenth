#ifndef FRENET_PLANNER_ROS_ROS_ADAPTERS_HPP
#define FRENET_PLANNER_ROS_ROS_ADAPTERS_HPP

#include "frenet_planner/core/types.hpp"
#include "frenet_planner/msg/planner_decision.hpp"
#include "frenet_planner/planning/local_planner.hpp"

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>

#include <string>

namespace frenet_planner
{

// ROS ↔ core type boundary.
Odometry rosToOdometry(const nav_msgs::msg::Odometry & msg);
OccupancyGrid rosToOccupancyGrid(const nav_msgs::msg::OccupancyGrid & msg);
nav_msgs::msg::Path pathToRos(
  const Path & path,
  const rclcpp::Time & stamp,
  const std::string & frame_id);
msg::PlannerDecision plannerDecisionToRos(
  const PlannerDecisionData & data,
  const rclcpp::Time & stamp,
  const std::string & frame_id);

} // namespace frenet_planner

#endif // FRENET_PLANNER_ROS_ROS_ADAPTERS_HPP
