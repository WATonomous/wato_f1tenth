#ifndef LOCAL_PLANNING_ROS_ROS_ADAPTERS_HPP
#define LOCAL_PLANNING_ROS_ROS_ADAPTERS_HPP

#include "local_planning/core/types.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace local_planning
{

// The one place ROS message types cross into the planner.  Everything below
// this line works on the internal types in core/types.hpp; nothing above it
// appears in the planning path.
Odometry rosToOdometry(const nav_msgs::msg::Odometry & msg);
OccupancyGrid rosToOccupancyGrid(const nav_msgs::msg::OccupancyGrid & msg);

} // namespace local_planning

#endif // LOCAL_PLANNING_ROS_ROS_ADAPTERS_HPP
