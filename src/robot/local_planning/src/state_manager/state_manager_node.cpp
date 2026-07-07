#include "planning/state_manager/state_manager_node.hpp"

#include "planning/ros_adapters.hpp"

#include <algorithm>
#include <functional>
#include <chrono>

namespace local_planning
{

using namespace std::placeholders;

namespace
{

const char * planIntentName(uint8_t intent)
{
  switch (intent) {
    case local_planning::action::PlanPath::Goal::OVERTAKE:
      return "OVERTAKE";
    case local_planning::action::PlanPath::Goal::MERGE:
      return "MERGE";
    case local_planning::action::PlanPath::Goal::FOLLOW_RACING_LINE:
      return "FOLLOW_RACING_LINE";
    default:
      return "UNKNOWN";
  }
}

} // namespace

StateManagerNode::StateManagerNode()
: Node("state_manager_node")
{
  // parameters
  this->declare_parameter<std::string>("racing_line_topic", "/global_planner/path");
  this->declare_parameter<double>("state_update_rate", 50.0);
  this->declare_parameter<double>("planning_trigger_rate", 50.0);
  this->declare_parameter<double>("overtake_start_distance_m", 3.0);
  this->declare_parameter<double>("side_by_side_distance_m", 0.5);
  this->declare_parameter<double>("merge_start_gap_m", 1.0);
  this->declare_parameter<double>("merge_done_gap_m", 2.0);
  this->declare_parameter<double>("merge_done_d_m", 0.25);

  racing_line_topic_ = this->get_parameter("racing_line_topic").as_string();
  state_update_rate_ = this->get_parameter("state_update_rate").as_double();
  planning_trigger_rate_ = this->get_parameter("planning_trigger_rate").as_double();
  overtake_start_distance_m_ = this->get_parameter("overtake_start_distance_m").as_double();
  side_by_side_distance_m_ = this->get_parameter("side_by_side_distance_m").as_double();
  merge_start_gap_m_ = this->get_parameter("merge_start_gap_m").as_double();
  merge_done_gap_m_ = this->get_parameter("merge_done_gap_m").as_double();
  merge_done_d_m_ = this->get_parameter("merge_done_d_m").as_double();

  // state machine
  state_machine_ = std::make_unique<RacingStateMachine>();
  state_machine_->setTransitionConfig(
    overtake_start_distance_m_,
    side_by_side_distance_m_,
    merge_start_gap_m_,
    merge_done_gap_m_,
    merge_done_d_m_);

  // subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&StateManagerNode::odometryCallback, this, _1));

  occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/occupancy_grid", 10,
    std::bind(&StateManagerNode::occupancyGridCallback, this, _1));

  racing_line_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    racing_line_topic_, rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&StateManagerNode::racingLineCallback, this, _1));

  // publishers
  state_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/racing_state", 10);

  // action client
  plan_action_client_ = rclcpp_action::create_client<PlanPath>(this, "plan_path");

  // timers
  auto state_period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / state_update_rate_));
  state_timer_ = this->create_wall_timer(
    state_period_ms,
    std::bind(&StateManagerNode::stateTimerCallback, this));

  const double safe_planning_trigger_rate = std::max(0.1, planning_trigger_rate_);
  planning_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / safe_planning_trigger_rate));
  planning_timer_ = this->create_wall_timer(
    planning_period_,
    std::bind(&StateManagerNode::planningTimerCallback, this));

}

void StateManagerNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg;
}

void StateManagerNode::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_occupancy_grid_ = msg;
}

void StateManagerNode::racingLineCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Ignoring empty racing line message on %s", racing_line_topic_.c_str());
    return;
  }

  std::vector<local_planning::Point> racing_line = rosPathToRacingLine(*msg);

  racing_line_ = std::move(racing_line);
  state_machine_->setRacingLine(racing_line_);
}

void StateManagerNode::stateTimerCallback()
{
  if (!current_odom_ || !current_occupancy_grid_) {
    return;
  }

  if (racing_line_.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Missing racing line on %s, skipping state update", racing_line_topic_.c_str());
    return;
  }

  local_planning::Odometry odom = rosToOdometry(*current_odom_);
  local_planning::OccupancyGrid grid = rosToOccupancyGrid(*current_occupancy_grid_);

  bool state_changed = state_machine_->update(odom, grid);

  if (state_changed) {
    publishState();
  }
}

void StateManagerNode::planningTimerCallback()
{
  if (!current_odom_ || !current_occupancy_grid_) {
    return;
  }

  if (racing_line_.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Missing racing line on %s, skipping plan trigger", racing_line_topic_.c_str());
    return;
  }

  sendPlanGoal(state_machine_->getCurrentState());
}

void StateManagerNode::scheduleNextPlanGoal()
{
  if (!current_odom_ || !current_occupancy_grid_) {
    return;
  }

  std::chrono::nanoseconds delay(0);
  if (has_sent_plan_goal_) {
    const auto next_send_time = last_plan_goal_sent_ + planning_period_;
    const auto now = std::chrono::steady_clock::now();
    if (next_send_time > now) {
      delay = std::chrono::duration_cast<std::chrono::nanoseconds>(next_send_time - now);
    }
  }

  if (deferred_planning_timer_) {
    deferred_planning_timer_->cancel();
  }

  if (delay <= std::chrono::milliseconds(1)) {
    planningTimerCallback();
    return;
  }

  deferred_planning_timer_ = this->create_wall_timer(
    delay,
    [this]() {
      if (deferred_planning_timer_) {
        deferred_planning_timer_->cancel();
      }
      planningTimerCallback();
    });
}

void StateManagerNode::publishState()
{
  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(state_machine_->getCurrentState());
  state_pub_->publish(msg);
  last_published_state_ = state_machine_->getCurrentState();
}

void StateManagerNode::sendPlanGoal(RacingState state)
{
  PlanPath::Goal goal_msg = buildPlanGoal(state);

  if (hasActivePlanGoal() || goal_request_pending_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Buffering plan goal while previous goal is active or pending (intent=%s, pending=%s)",
      planIntentName(goal_msg.intent), goal_request_pending_ ? "true" : "false");
    bufferPlanGoal(goal_msg);
    return;
  }

  if (!plan_action_server_ready_) {
    plan_action_server_ready_ = plan_action_client_->wait_for_action_server(
      std::chrono::milliseconds(100));
    if (!plan_action_server_ready_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Plan action server not available");
      return;
    }
  }

  const auto now = std::chrono::steady_clock::now();
  if (has_sent_plan_goal_ && now - last_plan_goal_sent_ < planning_period_) {
    return;
  }

  dispatchPlanGoal(goal_msg);
}

StateManagerNode::PlanPath::Goal StateManagerNode::buildPlanGoal(RacingState state) const
{
  auto goal_msg = PlanPath::Goal();

  switch (state) {
    case RacingState::BEHIND_OPPONENT:
    case RacingState::SIDE_BY_SIDE:
      goal_msg.intent = PlanPath::Goal::OVERTAKE;
      break;
    case RacingState::AHEAD_OPPONENT:
      goal_msg.intent = PlanPath::Goal::MERGE;
      break;
    case RacingState::STEADY_STATE:
    default:
      goal_msg.intent = PlanPath::Goal::FOLLOW_RACING_LINE;
      break;
  }

  return goal_msg;
}

void StateManagerNode::dispatchPlanGoal(const PlanPath::Goal & goal_msg)
{
  if (deferred_planning_timer_) {
    deferred_planning_timer_->cancel();
  }

  auto send_goal_options = rclcpp_action::Client<PlanPath>::SendGoalOptions();
  const uint8_t requested_intent = goal_msg.intent;
  send_goal_options.goal_response_callback =
    [this, requested_intent](const GoalHandle::SharedPtr & goal_handle) {
      goal_request_pending_ = false;

      if (!goal_handle) {
        RCLCPP_WARN(
          this->get_logger(), "Plan goal was rejected (intent=%s, buffered=%s)",
          planIntentName(requested_intent), buffered_plan_goal_ ? "true" : "false");
        if (!promoteBufferedPlanGoal()) {
          scheduleNextPlanGoal();
        }
        return;
      }

      current_goal_handle_ = goal_handle;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Plan goal accepted (intent=%s)", planIntentName(requested_intent));
      if (!hasActivePlanGoal()) {
        if (!promoteBufferedPlanGoal()) {
          scheduleNextPlanGoal();
        }
      }
    };
  send_goal_options.result_callback =
    std::bind(&StateManagerNode::planResultCallback, this, _1);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Dispatching plan goal (intent=%s, buffered=%s)",
    planIntentName(goal_msg.intent), buffered_plan_goal_ ? "true" : "false");

  goal_request_pending_ = true;
  last_plan_goal_sent_ = std::chrono::steady_clock::now();
  has_sent_plan_goal_ = true;
  plan_action_client_->async_send_goal(goal_msg, send_goal_options);
}

void StateManagerNode::bufferPlanGoal(const PlanPath::Goal & goal_msg)
{
  const bool replaced_existing_goal = buffered_plan_goal_.has_value();
  buffered_plan_goal_ = goal_msg;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Buffered plan goal (intent=%s, replaced_existing=%s)",
    planIntentName(goal_msg.intent), replaced_existing_goal ? "true" : "false");
}

bool StateManagerNode::hasActivePlanGoal()
{
  if (!current_goal_handle_ && !goal_request_pending_) {
    return false;
  }

  // if the planner died mid-goal the result callback never fires
  // so an in-flight goal eventually is presumed lost.
  const auto goal_age = std::chrono::steady_clock::now() - last_plan_goal_sent_;
  if (goal_age > 3 * planning_period_) {
    current_goal_handle_ = nullptr;
    goal_request_pending_ = false;
    plan_action_server_ready_ = false;
    return false;
  }

  if (!current_goal_handle_) {
    return false;
  }

  try {
    const int8_t status = current_goal_handle_->get_status();
    if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
      status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELING)
    {
      return true;
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not inspect current plan goal state: %s", ex.what());
    return true;
  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "Could not inspect current plan goal state");
    return true;
  }

  current_goal_handle_ = nullptr;
  return false;
}

bool StateManagerNode::promoteBufferedPlanGoal()
{
  if (!buffered_plan_goal_ || hasActivePlanGoal() || goal_request_pending_) {
    return false;
  }

  PlanPath::Goal next_goal = *buffered_plan_goal_;
  buffered_plan_goal_.reset();

  if (!plan_action_server_ready_) {
    plan_action_server_ready_ = plan_action_client_->wait_for_action_server(
      std::chrono::milliseconds(100));
    if (!plan_action_server_ready_) {
      buffered_plan_goal_ = next_goal;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Plan action server not available");
      return false;
    }
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Promoting buffered plan goal (intent=%s)", planIntentName(next_goal.intent));
  dispatchPlanGoal(next_goal);
  return true;
}

void StateManagerNode::planResultCallback(const GoalHandle::WrappedResult & result)
{
  if (current_goal_handle_ && result.goal_id != current_goal_handle_->get_goal_id()) {
    return;
  }
  current_goal_handle_ = nullptr;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (!result.result->success) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Planner returned failure (path_points=%zu, buffered=%s)",
          result.result->path.poses.size(), buffered_plan_goal_ ? "true" : "false");
      } else {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Planner returned success (path_points=%zu, buffered=%s)",
          result.result->path.poses.size(), buffered_plan_goal_ ? "true" : "false");
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(this->get_logger(), "Plan goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Planner goal ended with unknown result code");
      break;
  }

  if (promoteBufferedPlanGoal()) {
    return;
  }

  scheduleNextPlanGoal();
}


} // namespace local_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planning::StateManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
