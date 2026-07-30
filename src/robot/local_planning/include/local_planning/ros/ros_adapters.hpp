#ifndef LOCAL_PLANNING_ROS_ROS_ADAPTERS_HPP
#define LOCAL_PLANNING_ROS_ROS_ADAPTERS_HPP

#include "local_planning/core/types.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <vector>

namespace local_planning
{

// The one place ROS message types cross into the planner.  Everything below
// this line works on the internal types in core/types.hpp; nothing above it
// appears in the planning path.
Odometry rosToOdometry(const nav_msgs::msg::Odometry & msg);
OccupancyGrid rosToOccupancyGrid(const nav_msgs::msg::OccupancyGrid & msg);

// The /racing_line topic contract: a nav_msgs::Path whose pose.position.z
// carries the raceline speed at that waypoint.  It is a convention rather than
// a typed field, and it is load-bearing — the PRD 12 terminal speed cap needs
// raceline_speed(s).  racing_line_publisher_node is the only place the raceline
// CSV is parsed; nothing else may re-read it.
std::vector<Point> rosPathToRacingLine(const nav_msgs::msg::Path & msg);

} // namespace local_planning

#endif // LOCAL_PLANNING_ROS_ROS_ADAPTERS_HPP
