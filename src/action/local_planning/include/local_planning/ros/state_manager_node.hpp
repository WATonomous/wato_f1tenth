#ifndef LOCAL_PLANNING_ROS_STATE_MANAGER_NODE_HPP
#define LOCAL_PLANNING_ROS_STATE_MANAGER_NODE_HPP

#include "local_planning/state/racing_state_machine.hpp"

#include <local_planning/msg/planner_intent.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <memory>
#include <string>
#include <vector>

namespace local_planning
{
// Thin wrapper around RacingStateMachine.  The state manager stays its own
// node so the planner sees intent as a message, but the class underneath is
// ROS-free so Phase 6 can table-test it without spinning anything.
class StateManagerNode : public rclcpp::Node
{
public:
  StateManagerNode();

private:
  void odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg);
  void occupancyGridCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void racingLineCallback(nav_msgs::msg::Path::SharedPtr msg);
  void stateTimerCallback();
  void publishStateAndIntent();
  uint8_t intentForState(RacingState state) const;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr racing_line_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::Publisher<msg::PlannerIntent>::SharedPtr intent_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  std::unique_ptr<RacingStateMachine> state_machine_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_occupancy_grid_;
  std::vector<Point> racing_line_;
  std::string racing_line_topic_;
};
}  // namespace local_planning
#endif  // LOCAL_PLANNING_ROS_STATE_MANAGER_NODE_HPP
