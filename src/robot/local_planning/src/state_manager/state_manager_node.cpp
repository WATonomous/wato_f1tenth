#include "planning/state_manager/state_manager_node.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <cmath>
#include <functional>
#include <chrono>

namespace local_planning
{

using namespace std::placeholders;

StateManagerNode::StateManagerNode()
: Node("state_manager_node")
{
  // parameters
  this->declare_parameter<std::string>("racing_line_file", "racing_line.csv");
  this->declare_parameter<double>("state_update_rate", 20.0);
  this->declare_parameter<double>("planning_trigger_rate", 10.0);
  this->declare_parameter<double>("overtake_start_distance_m", 3.0);
  this->declare_parameter<double>("side_by_side_distance_m", 0.5);
  this->declare_parameter<double>("merge_start_gap_m", 1.0);
  this->declare_parameter<double>("merge_done_gap_m", 2.0);
  this->declare_parameter<double>("merge_done_d_m", 0.25);
  this->declare_parameter<int>("num_lattices", 5);
  this->declare_parameter<double>("lattice_spacing", 0.1);

  racing_line_file_ = this->get_parameter("racing_line_file").as_string();
  state_update_rate_ = this->get_parameter("state_update_rate").as_double();
  planning_trigger_rate_ = this->get_parameter("planning_trigger_rate").as_double();
  overtake_start_distance_m_ = this->get_parameter("overtake_start_distance_m").as_double();
  side_by_side_distance_m_ = this->get_parameter("side_by_side_distance_m").as_double();
  merge_start_gap_m_ = this->get_parameter("merge_start_gap_m").as_double();
  merge_done_gap_m_ = this->get_parameter("merge_done_gap_m").as_double();
  merge_done_d_m_ = this->get_parameter("merge_done_d_m").as_double();
  num_lattices_ = this->get_parameter("num_lattices").as_int();
  lattice_spacing_ = this->get_parameter("lattice_spacing").as_double();

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

  // publishers
  state_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/racing_state", 10);
  lattice_viz_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/lattice_viz", 10);

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

  loadRacingLine();

  RCLCPP_INFO(this->get_logger(), "StateManagerNode initialized");
}

void StateManagerNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg;
}

void StateManagerNode::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_occupancy_grid_ = msg;
}

void StateManagerNode::stateTimerCallback()
{
  if (!current_odom_ || !current_occupancy_grid_) {
    return;
  }

  local_planning::Odometry odom = rosToOdometry(current_odom_);
  local_planning::OccupancyGrid grid = rosToOccupancyGrid(current_occupancy_grid_);

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

void StateManagerNode::loadRacingLine()
{
  racing_line_ = loadRacingLineFromFile(racing_line_file_);

  if (racing_line_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to load racing line from: %s",
      racing_line_file_.c_str());
    return;
  }

  state_machine_->setRacingLine(racing_line_);
  RCLCPP_INFO(this->get_logger(), "Loaded racing line with %zu points", racing_line_.size());

  publishRacingLineLattices();
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

  if (current_goal_handle_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (has_sent_plan_goal_ && now - last_plan_goal_sent_ < planning_period_) {
    return;
  }

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

  auto send_goal_options = rclcpp_action::Client<PlanPath>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const GoalHandle::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_WARN(this->get_logger(), "Plan goal was rejected");
        return;
      }
      current_goal_handle_ = goal_handle;
    };
  send_goal_options.result_callback =
    std::bind(&StateManagerNode::planResultCallback, this, _1);

  last_plan_goal_sent_ = std::chrono::steady_clock::now();
  has_sent_plan_goal_ = true;
  plan_action_client_->async_send_goal(goal_msg, send_goal_options);
}

void StateManagerNode::planResultCallback(const GoalHandle::WrappedResult & result)
{
  current_goal_handle_ = nullptr;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (!result.result->success) {
        RCLCPP_WARN(this->get_logger(), "Planner returned failure");
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(this->get_logger(), "Plan goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "???");
      break;
  }

  scheduleNextPlanGoal();
}
//visualizer stuff
void StateManagerNode::publishRacingLineLattices()
{
  if (racing_line_.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  // racing line
  visualization_msgs::msg::Marker line_marker;
  line_marker.header.frame_id = "map";
  line_marker.header.stamp = this->now();
  line_marker.ns = "racing_line";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.scale.x = 0.03;
  line_marker.color.r = 1.0;
  line_marker.color.g = 0.0;
  line_marker.color.b = 0.0;
  line_marker.color.a = 1.0;

  for (const auto & p : racing_line_) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = 0.0;
    line_marker.points.push_back(pt);
  }

  // close the loop
  if (racing_line_.size() > 1) {
    geometry_msgs::msg::Point pt;
    pt.x = racing_line_.front().x;
    pt.y = racing_line_.front().y;
    pt.z = 0.0;
    line_marker.points.push_back(pt);
  }

  markers.markers.push_back(line_marker);

  // offset lattice lines
  int marker_id = 1;
  int n = static_cast<int>(racing_line_.size());
  for (int l = -num_lattices_; l <= num_lattices_; ++l) {
    if (l == 0) {
      continue;
    }

    double offset = l * lattice_spacing_;

    visualization_msgs::msg::Marker lattice_marker;
    lattice_marker.header.frame_id = "map";
    lattice_marker.header.stamp = this->now();
    lattice_marker.ns = "lattice_lines";
    lattice_marker.id = marker_id++;
    lattice_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    lattice_marker.action = visualization_msgs::msg::Marker::ADD;
    lattice_marker.scale.x = 0.02;
    lattice_marker.color.r = 0.5;
    lattice_marker.color.g = 0.5;
    lattice_marker.color.b = 1.0;
    lattice_marker.color.a = 0.5;

    for (int i = 0; i < n; ++i) {
      int next = (i + 1) % n;
      double dx = racing_line_[next].x - racing_line_[i].x;
      double dy = racing_line_[next].y - racing_line_[i].y;
      double len = std::hypot(dx, dy);
      if (len < 1e-12) {
        continue;
      }
      // perpendicular normal (left-pointing)
      double nx = -dy / len;
      double ny = dx / len;

      geometry_msgs::msg::Point pt;
      pt.x = racing_line_[i].x + offset * nx;
      pt.y = racing_line_[i].y + offset * ny;
      pt.z = 0.0;
      lattice_marker.points.push_back(pt);
    }

    // close the loop
    if (!lattice_marker.points.empty()) {
      lattice_marker.points.push_back(lattice_marker.points.front());
    }

    markers.markers.push_back(lattice_marker);
  }

  lattice_viz_pub_->publish(markers);
}

std::vector<local_planning::Point> StateManagerNode::loadRacingLineFromFile(
  const std::string & filename)
{
  std::vector<local_planning::Point> points;
  std::ifstream file(filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to open racing line file: %s",
      filename.c_str());
    return points;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    double x, y, v;
    char comma;
    if (ss >> x >> comma >> y >> comma >> v) {
      points.emplace_back(x, y, v);
    }
  }

  return points;
}

local_planning::Odometry StateManagerNode::rosToOdometry(
  const nav_msgs::msg::Odometry::SharedPtr & msg)
{
  local_planning::Odometry odom;
  odom.position.x = msg->pose.pose.position.x;
  odom.position.y = msg->pose.pose.position.y;
  odom.velocity = msg->twist.twist.linear.x;
  odom.heading = tf2::getYaw(msg->pose.pose.orientation);
  return odom;
}

local_planning::OccupancyGrid StateManagerNode::rosToOccupancyGrid(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & msg)
{
  local_planning::OccupancyGrid grid;
  grid.width = static_cast<int>(msg->info.width);
  grid.height = static_cast<int>(msg->info.height);
  grid.resolution = msg->info.resolution;
  grid.origin.x = msg->info.origin.position.x;
  grid.origin.y = msg->info.origin.position.y;
  grid.data.assign(msg->data.begin(), msg->data.end());
  return grid;
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
