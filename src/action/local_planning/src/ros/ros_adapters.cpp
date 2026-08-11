#include "local_planning/ros/ros_adapters.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace local_planning
{

Odometry rosToOdometry(const nav_msgs::msg::Odometry & msg)
{
  Odometry odom;
  odom.position.x = msg.pose.pose.position.x;
  odom.position.y = msg.pose.pose.position.y;
  odom.velocity = msg.twist.twist.linear.x;
  odom.heading = tf2::getYaw(msg.pose.pose.orientation);
  return odom;
}

OccupancyGrid rosToOccupancyGrid(const nav_msgs::msg::OccupancyGrid & msg)
{
  OccupancyGrid grid;
  grid.width = static_cast<int>(msg.info.width);
  grid.height = static_cast<int>(msg.info.height);
  grid.resolution = msg.info.resolution;
  grid.origin.x = msg.info.origin.position.x;
  grid.origin.y = msg.info.origin.position.y;
  grid.data.assign(msg.data.begin(), msg.data.end());
  return grid;
}

} // namespace local_planning
