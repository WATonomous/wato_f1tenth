#ifndef PLANNING_STATE_MANAGER_NODE_HPP
#define PLANNING_STATE_MANAGER_NODE_HPP

#include "planning/state_manager/state_manager_code.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <local_planning/action/plan_path.hpp>

#include <chrono>
#include <memory>
#include <vector>
#include <string>

namespace local_planning
{

class StateManagerNode : public rclcpp::Node
{
public:
  StateManagerNode();
  ~StateManagerNode() = default;

private:
  using PlanPath = local_planning::action::PlanPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<PlanPath>;

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  void stateTimerCallback();
  void planningTimerCallback();
  void scheduleNextPlanGoal();

  //utility
  void loadRacingLine();
  void publishState();
  void sendPlanGoal(RacingState state);

  //mainly for visualization in foxglove
  void publishRacingLineLattices();

  // action result callback
  void planResultCallback(const GoalHandle::WrappedResult & result);

  // conversions
  local_planning::Odometry rosToOdometry(const nav_msgs::msg::Odometry::SharedPtr & msg);
  local_planning::OccupancyGrid rosToOccupancyGrid(
    const nav_msgs::msg::OccupancyGrid::SharedPtr & msg);
  std::vector<local_planning::Point> loadRacingLineFromFile(const std::string & filename);

  // subs
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;

  // pubs
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lattice_viz_pub_;

  // action client
  rclcpp_action::Client<PlanPath>::SharedPtr plan_action_client_;

  // timers
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  // state machine
  std::unique_ptr<RacingStateMachine> state_machine_;

  // cached messages
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_occupancy_grid_;

  // racing line
  std::vector<local_planning::Point> racing_line_;

  // parameters
  std::string racing_line_file_;
  double state_update_rate_;
  double planning_trigger_rate_;
  double overtake_start_distance_m_;
  double side_by_side_distance_m_;
  double merge_start_gap_m_;
  double merge_done_gap_m_;
  double merge_done_d_m_;
  int num_lattices_;
  double lattice_spacing_;

  // state tracking
  RacingState last_published_state_{RacingState::STEADY_STATE};

  // track in-flight action goal so we do not churn/cancel the synchronous planner
  GoalHandle::SharedPtr current_goal_handle_;
  bool plan_action_server_ready_{false};
  std::chrono::nanoseconds planning_period_{std::chrono::milliseconds(100)};
  std::chrono::steady_clock::time_point last_plan_goal_sent_{};
  bool has_sent_plan_goal_{false};
  rclcpp::TimerBase::SharedPtr deferred_planning_timer_;
};

} // namespace local_planning

#endif // PLANNING_STATE_MANAGER_NODE_HPP
