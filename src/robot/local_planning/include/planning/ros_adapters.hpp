#ifndef PLANNING_ROS_ADAPTERS_HPP
#define PLANNING_ROS_ADAPTERS_HPP

#include "planning/types.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <vector>

namespace local_planning
{

Odometry rosToOdometry(const nav_msgs::msg::Odometry & msg);
OccupancyGrid rosToOccupancyGrid(const nav_msgs::msg::OccupancyGrid & msg);
std::vector<Point> rosPathToRacingLine(const nav_msgs::msg::Path & msg);

} // namespace local_planning

#endif // PLANNING_ROS_ADAPTERS_HPP
