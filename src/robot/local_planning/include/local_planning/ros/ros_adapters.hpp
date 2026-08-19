#ifndef LOCAL_PLANNING_ROS_ROS_ADAPTERS_HPP
#define LOCAL_PLANNING_ROS_ROS_ADAPTERS_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/msg/planner_decision.hpp"
#include "local_planning/planning/local_planner.hpp"

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>

#include <string>

namespace local_planning
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

} // namespace local_planning

#endif // LOCAL_PLANNING_ROS_ROS_ADAPTERS_HPP
