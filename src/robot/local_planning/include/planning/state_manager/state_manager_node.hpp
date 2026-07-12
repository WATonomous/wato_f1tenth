#ifndef PLANNING_STATE_MANAGER_NODE_HPP
#define PLANNING_STATE_MANAGER_NODE_HPP

#include "planning/state_manager/state_manager_code.hpp"

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
#endif
