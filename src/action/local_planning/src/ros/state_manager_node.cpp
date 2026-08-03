#include "local_planning/ros/state_manager_node.hpp"

#include "local_planning/ros/ros_adapters.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

namespace local_planning
{
using namespace std::placeholders;

StateManagerNode::StateManagerNode()
: Node("state_manager_node")
{
  declare_parameter<std::string>("racing_line_topic", "/racing_line");
  declare_parameter<double>("state_update_rate", 50.0);
  declare_parameter<double>("overtake_start_distance_m", 3.0);
  declare_parameter<double>("side_by_side_distance_m", 0.5);
  declare_parameter<double>("merge_start_gap_m", 1.0);
  declare_parameter<double>("merge_done_gap_m", 2.0);
  declare_parameter<double>("merge_done_d_m", 0.4);
  racing_line_topic_ = get_parameter("racing_line_topic").as_string();

  state_machine_ = std::make_unique<RacingStateMachine>();
  state_machine_->setTransitionConfig(
    get_parameter("overtake_start_distance_m").as_double(),
    get_parameter("side_by_side_distance_m").as_double(),
    get_parameter("merge_start_gap_m").as_double(),
    get_parameter("merge_done_gap_m").as_double(),
    get_parameter("merge_done_d_m").as_double());

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 1, std::bind(&StateManagerNode::odometryCallback, this, _1));
  occupancy_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/occupancy_grid", 1, std::bind(&StateManagerNode::occupancyGridCallback, this, _1));
  racing_line_sub_ = create_subscription<nav_msgs::msg::Path>(
    racing_line_topic_, rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&StateManagerNode::racingLineCallback, this, _1));
  state_pub_ = create_publisher<std_msgs::msg::UInt8>("/racing_state", 10);
  intent_pub_ = create_publisher<msg::PlannerIntent>(
    "/planner_intent", rclcpp::QoS(1).transient_local().reliable());

  const double rate = std::max(0.1, get_parameter("state_update_rate").as_double());
  state_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate),
    std::bind(&StateManagerNode::stateTimerCallback, this));
  publishStateAndIntent();
}

void StateManagerNode::odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{current_odom_ = std::move(msg);}

void StateManagerNode::occupancyGridCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{current_occupancy_grid_ = std::move(msg);}

void StateManagerNode::racingLineCallback(nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {return;}
  racing_line_ = rosPathToRacingLine(*msg);
  if (!state_machine_->setRacingLine(racing_line_)) {
    RCLCPP_ERROR(get_logger(), "Racing line rejected: too few or degenerate waypoints");
    racing_line_.clear();
  }
}

void StateManagerNode::stateTimerCallback()
{
  if (!current_odom_ || !current_occupancy_grid_ || racing_line_.empty()) {return;}
  if (state_machine_->update(
      rosToOdometry(*current_odom_), rosToOccupancyGrid(*current_occupancy_grid_)))
  {
    publishStateAndIntent();
  }
}

// PASS is not reachable yet: the carried-forward state machine has no
// side-by-side-to-clear distinction.  Phase 6 adds it along with the
// presence timeout and the merge dwell latch.
uint8_t StateManagerNode::intentForState(RacingState state) const
{
  switch (state) {
    case RacingState::BEHIND_OPPONENT:
    case RacingState::SIDE_BY_SIDE: return msg::PlannerIntent::OVERTAKE;
    case RacingState::AHEAD_OPPONENT: return msg::PlannerIntent::MERGE;
    default: return msg::PlannerIntent::FOLLOW_RACING_LINE;
  }
}

void StateManagerNode::publishStateAndIntent()
{
  const RacingState state = state_machine_->getCurrentState();
  std_msgs::msg::UInt8 state_msg; state_msg.data = static_cast<uint8_t>(state);
  state_pub_->publish(state_msg);
  msg::PlannerIntent intent; intent.header.stamp = now();
  intent.header.frame_id = racing_line_topic_; intent.intent = intentForState(state);
  intent_pub_->publish(intent);
}
}  // namespace local_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<local_planning::StateManagerNode>());
  rclcpp::shutdown(); return 0;
}
