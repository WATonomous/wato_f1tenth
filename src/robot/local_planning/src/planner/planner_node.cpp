#include "planning/planner/planner_node.hpp"

#include "planning/planner/local_frenet_lattice_planner.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fstream>
#include <sstream>
#include <thread>
#include <functional>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace local_planning
{

using namespace std::placeholders;

namespace
{

using SteadyClock = std::chrono::steady_clock;

constexpr double kMinimumPlannerBudgetMs = 1.0;
constexpr double kSearchBudgetReserveMs = 20.0;
constexpr double kVizDeadlineReserveMs = 5.0;

class ScopedPlannerBusyFlag
{
public:
  explicit ScopedPlannerBusyFlag(std::atomic_bool & flag)
  : flag_(flag)
  {
  }

  ~ScopedPlannerBusyFlag()
  {
    flag_.store(false);
  }

private:
  std::atomic_bool & flag_;
};

double elapsedMs(SteadyClock::time_point start, SteadyClock::time_point end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

double elapsedMs(SteadyClock::time_point start)
{
  return elapsedMs(start, SteadyClock::now());
}

} // namespace

PlannerNode::PlannerNode()
: Node("hybrid_astar_planner_node")
{
  // parameters check yaml for explanation
  this->declare_parameter<std::string>("racing_line_file", "racing_line.csv");
  this->declare_parameter<int>(
    "default_intent",
    static_cast<int>(LocalPlannerIntent::FOLLOW_RACING_LINE));
  this->declare_parameter<double>("horizon_m", 6.0);
  this->declare_parameter<double>("layer_spacing_m", 0.5);
  this->declare_parameter<double>("lane_spacing_m", 0.1);
  this->declare_parameter<double>("max_lateral_offset_m", 1.8);
  this->declare_parameter<int>("max_lane_jump_per_layer", 3);
  this->declare_parameter<double>("max_path_angle_deg", 50.0);
  this->declare_parameter<double>("sample_spacing_m", 0.1);
  this->declare_parameter<double>("max_runtime_ms", 50.0);
  this->declare_parameter<double>("heuristic_weight", 1.0);
  this->declare_parameter<std::vector<double>>(
    "heading_buckets_deg", std::vector<double>{-10.0, -5.0, 0.0, 5.0, 10.0});
  this->declare_parameter<int>("max_heading_jump_per_layer", 1);
  this->declare_parameter<double>("max_heading_mismatch_deg", 25.0);
  this->declare_parameter<int>("heuristic_sample_count", 8);
  this->declare_parameter<double>("collision_circle_radius_m", 0.20);
  this->declare_parameter<double>("front_collision_circle_offset_m", 0.26);
  this->declare_parameter<double>("soft_inflation_distance_m", 0.18);
  this->declare_parameter<double>("soft_inflation_cost", 100.0);
  this->declare_parameter<int>("occupied_threshold", 50);
  this->declare_parameter<double>("friction_coeff", 1.0);
  this->declare_parameter<double>("min_velocity_mps", 0.5);
  this->declare_parameter<double>("max_velocity_mps", 10.0);
  this->declare_parameter<double>("time_weight", 1.0);
  this->declare_parameter<double>("curvature_change_weight", 0.4);
  this->declare_parameter<double>("follow_d_weight", 0.20);
  this->declare_parameter<double>("overtake_d_weight", 0.02);
  this->declare_parameter<double>("merge_d_weight", 0.20);
  this->declare_parameter<double>("merge_terminal_d_weight", 0.0);
  this->declare_parameter<double>("planner_runtime_budget_ms", 100.0);
  this->declare_parameter<std::string>("planner_path_frame", "map");
  this->declare_parameter<std::string>("controller_path_frame", "base_link");
  this->declare_parameter<std::string>("debug_path_topic", "/local_path_map");

  racing_line_file_ = this->get_parameter("racing_line_file").as_string();
  switch (std::clamp(static_cast<int>(this->get_parameter("default_intent").as_int()), 0, 2)) {
    case 1:
      default_intent_ = LocalPlannerIntent::OVERTAKE;
      break;
    case 2:
      default_intent_ = LocalPlannerIntent::MERGE;
      break;
    case 0:
    default:
      default_intent_ = LocalPlannerIntent::FOLLOW_RACING_LINE;
      break;
  }
  planner_config_.horizon_m = this->get_parameter("horizon_m").as_double();
  planner_config_.layer_spacing_m = this->get_parameter("layer_spacing_m").as_double();
  planner_config_.lane_spacing_m = this->get_parameter("lane_spacing_m").as_double();
  planner_config_.max_lateral_offset_m = this->get_parameter("max_lateral_offset_m").as_double();
  planner_config_.max_lane_jump_per_layer = this->get_parameter(
    "max_lane_jump_per_layer").as_int();
  planner_config_.max_path_angle_deg = this->get_parameter("max_path_angle_deg").as_double();
  planner_config_.sample_spacing_m = this->get_parameter("sample_spacing_m").as_double();
  planner_config_.max_runtime_ms = this->get_parameter("max_runtime_ms").as_double();
  planner_config_.heuristic_weight = this->get_parameter("heuristic_weight").as_double();
  planner_config_.heading_buckets_deg =
    this->get_parameter("heading_buckets_deg").as_double_array();
  planner_config_.max_heading_jump_per_layer = this->get_parameter(
    "max_heading_jump_per_layer").as_int();
  planner_config_.max_heading_mismatch_deg = this->get_parameter(
    "max_heading_mismatch_deg").as_double();
  planner_config_.heuristic_sample_count = this->get_parameter("heuristic_sample_count").as_int();
  planner_config_.collision_circle_radius_m = this->get_parameter(
    "collision_circle_radius_m").as_double();
  planner_config_.front_collision_circle_offset_m = this->get_parameter(
    "front_collision_circle_offset_m").as_double();
  planner_config_.soft_inflation_distance_m = this->get_parameter(
    "soft_inflation_distance_m").as_double();
  planner_config_.soft_inflation_cost = this->get_parameter("soft_inflation_cost").as_double();
  planner_config_.occupied_threshold = this->get_parameter("occupied_threshold").as_int();
  planner_config_.friction_coeff = this->get_parameter("friction_coeff").as_double();
  planner_config_.min_velocity_mps = this->get_parameter("min_velocity_mps").as_double();
  planner_config_.max_velocity_mps = this->get_parameter("max_velocity_mps").as_double();
  planner_config_.time_weight = this->get_parameter("time_weight").as_double();
  planner_config_.curvature_change_weight =
    this->get_parameter("curvature_change_weight").as_double();
  planner_config_.follow_d_weight = this->get_parameter("follow_d_weight").as_double();
  planner_config_.overtake_d_weight = this->get_parameter("overtake_d_weight").as_double();
  planner_config_.merge_d_weight = this->get_parameter("merge_d_weight").as_double();
  planner_config_.merge_terminal_d_weight =
    this->get_parameter("merge_terminal_d_weight").as_double();
  planner_runtime_budget_ms_ = std::max(
    kMinimumPlannerBudgetMs, this->get_parameter("planner_runtime_budget_ms").as_double());
  planner_path_frame_ = this->get_parameter("planner_path_frame").as_string();
  controller_path_frame_ = this->get_parameter("controller_path_frame").as_string();
  debug_path_topic_ = this->get_parameter("debug_path_topic").as_string();

  const double max_search_budget_ms = std::max(
    kMinimumPlannerBudgetMs, planner_runtime_budget_ms_ - kSearchBudgetReserveMs);
  if (planner_config_.max_runtime_ms > max_search_budget_ms) {
    RCLCPP_WARN(
      this->get_logger(),
      "Clamping planner search budget from %.2f ms to %.2f ms to keep action budget %.2f ms",
      planner_config_.max_runtime_ms, max_search_budget_ms, planner_runtime_budget_ms_);
    planner_config_.max_runtime_ms = max_search_budget_ms;
  }

  // planner
  planner_ = std::make_unique<LocalFrenetLatticePlanner>();
  planner_->setConfig(planner_config_);

  // tf2
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&PlannerNode::odometryCallback, this, _1));

  occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/occupancy_grid", 10,
    std::bind(&PlannerNode::occupancyGridCallback, this, _1));

  // publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  debug_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(debug_path_topic_, 10);
  viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/local_frenet_lattice_viz", 10);

  // action server
  plan_action_server_ = rclcpp_action::create_server<PlanPath>(
    this,
    "plan_path",
    std::bind(&PlannerNode::handleGoal, this, _1, _2),
    std::bind(&PlannerNode::handleCancel, this, _1),
    std::bind(&PlannerNode::handleAccepted, this, _1));

  loadRacingLine();

}
//mutex this to prevent races
void PlannerNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(input_mutex_);
  current_odom_ = msg;
}

void PlannerNode::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  local_planning::OccupancyGrid grid = rosToOccupancyGrid(msg);
  buildClearanceMasks(grid);

  std::lock_guard<std::mutex> lock(input_mutex_);
  current_occupancy_grid_ = std::move(grid);
  has_current_occupancy_grid_ = true;
}

rclcpp_action::GoalResponse PlannerNode::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PlanPath::Goal>/*goal*/)
{
  bool expected_idle = false;
  if (!planner_busy_.compare_exchange_strong(expected_idle, true)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Rejecting plan goal because the previous plan is still running");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerNode::handleCancel(
  const std::shared_ptr<GoalHandle>/*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerNode::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  const uint64_t sequence = ++latest_plan_sequence_;
  auto self = shared_from_this();
  auto goal_handle_holder = new std::shared_ptr<GoalHandle>(goal_handle);
  std::thread(
    [this, self, goal_handle_holder, sequence]() {
      try {
        executePlan(*goal_handle_holder, sequence);
      } catch (const std::exception & ex) {
        planner_busy_.store(false);
        if (rclcpp::ok()) {
          RCLCPP_WARN(
            this->get_logger(), "Planner action worker exited after exception: %s", ex.what());
        }
      } catch (...) {
        planner_busy_.store(false);
        if (rclcpp::ok()) {
          RCLCPP_WARN(this->get_logger(), "Planner action worker exited after unknown exception");
        }
      }

      bool release_goal_handle = rclcpp::ok();
      try {
        release_goal_handle = release_goal_handle && !(*goal_handle_holder)->is_active();
      } catch (...) {
        release_goal_handle = false;
      }

      if (release_goal_handle) {
        delete goal_handle_holder;
      }
    }).detach();
}

void PlannerNode::executePlan(const std::shared_ptr<GoalHandle> goal_handle, uint64_t sequence)
try
{
  ScopedPlannerBusyFlag clear_busy(planner_busy_);
  const auto total_start = SteadyClock::now();
  auto result = std::make_shared<PlanPath::Result>();

  auto goal = goal_handle->get_goal();
  const LocalPlannerIntent intent = intentFromAction(goal->intent);

  auto action_budget_expired = [&]() {
      return elapsedMs(total_start) >= planner_runtime_budget_ms_;
    };
  auto has_budget_for_viz = [&]() {
      return elapsedMs(total_start) + kVizDeadlineReserveMs < planner_runtime_budget_ms_;
    };

  auto ros_active = []() {
      return rclcpp::ok();
    };

  auto finish_succeeded = [&](const std::shared_ptr<PlanPath::Result> & result_msg) {
      if (!ros_active()) {
        return;
      }
      try {
        if (!goal_handle->is_active()) {
          return;
        }
        goal_handle->succeed(result_msg);
      } catch (const std::exception & ex) {
        if (rclcpp::ok()) {
          RCLCPP_WARN(
            this->get_logger(), "Could not publish planning result because goal ended: %s",
            ex.what());
        }
      } catch (...) {
        if (rclcpp::ok()) {
          RCLCPP_WARN(this->get_logger(), "Could not publish planning result because goal ended");
        }
      }
    };

  auto finish_canceled = [&](const std::shared_ptr<PlanPath::Result> & result_msg) {
      if (!ros_active()) {
        return;
      }
      try {
        if (!goal_handle->is_active()) {
          return;
        }
        goal_handle->canceled(result_msg);
      } catch (const std::exception & ex) {
        if (rclcpp::ok()) {
          RCLCPP_WARN(
            this->get_logger(), "Could not publish canceled planning result because goal ended: %s",
            ex.what());
        }
      } catch (...) {
        if (rclcpp::ok()) {
          RCLCPP_WARN(this->get_logger(), "Could not publish canceled planning result because goal ended");
        }
      }
    };

  nav_msgs::msg::Odometry::SharedPtr odom_msg;
  local_planning::OccupancyGrid occupancy_grid;
  bool has_occupancy_grid = false;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    odom_msg = current_odom_;
    if (has_current_occupancy_grid_) {
      occupancy_grid = current_occupancy_grid_;
      has_occupancy_grid = true;
    }
  }

  if (!odom_msg || !has_occupancy_grid) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Missing odom or occupancy grid, cannot plan");
    result->success = false;
    finish_succeeded(result);
    return;
  }

  if (goal_handle->is_canceling()) {
    result->success = false;
    finish_canceled(result);
    return;
  }

  local_planning::Odometry odom = rosToOdometry(odom_msg);

  if (action_budget_expired()) {
    result->success = false;
    finish_succeeded(result);
    return;
  }

  LocalFrenetPlan plan = planner_->plan(odom.position, odom.heading, occupancy_grid, intent);

  if (!ros_active()) {
    return;
  }

  const nav_msgs::msg::Path planner_path = pathToRosPath(plan.path, planner_path_frame_);

  const bool stale_plan = sequence != latest_plan_sequence_.load();
  const bool over_budget = action_budget_expired();
  if (stale_plan || goal_handle->is_canceling() || over_budget) {
    result->success = false;
    if (goal_handle->is_canceling()) {
      finish_canceled(result);
    } else {
      finish_succeeded(result);
    }
    return;
  }

  if (plan.path.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Planner returned empty path");
    result->success = false;
    result->path = pathToRosPath(plan.path, controller_path_frame_);
    std::lock_guard<std::mutex> lock(publish_mutex_);
    const bool stale_or_canceling = sequence != latest_plan_sequence_.load() ||
      goal_handle->is_canceling();
    if (stale_or_canceling) {
      if (goal_handle->is_canceling()) {
        finish_canceled(result);
      } else {
        finish_succeeded(result);
      }
      return;
    }

    path_pub_->publish(result->path);
    debug_path_pub_->publish(planner_path);
    if (has_budget_for_viz()) {
      publishPlannerViz(plan);
    }
    finish_succeeded(result);
    return;
  }

  nav_msgs::msg::Path controller_path;
  if (!transformPathToControllerFrame(planner_path, controller_path)) {
    result->success = false;
    result->path = pathToRosPath({}, controller_path_frame_);
    finish_succeeded(result);
    return;
  }

  result->path = controller_path;
  result->success = true;

  std::lock_guard<std::mutex> lock(publish_mutex_);
  const bool stale_or_canceling = sequence != latest_plan_sequence_.load() ||
    goal_handle->is_canceling();
  if (stale_or_canceling || action_budget_expired()) {
    result->success = false;
    if (goal_handle->is_canceling()) {
      finish_canceled(result);
    } else {
      finish_succeeded(result);
    }
    return;
  }

  if (!ros_active()) {
    return;
  }

  path_pub_->publish(result->path);
  debug_path_pub_->publish(planner_path);

  if (has_budget_for_viz()) {
    publishPlannerViz(plan);
  }

  result->success = !action_budget_expired();
  finish_succeeded(result);
}
catch (const std::exception & ex)
{
  planner_busy_.store(false);
  if (rclcpp::ok()) {
    RCLCPP_WARN(this->get_logger(), "Planner action thread exited after exception: %s", ex.what());
  }
}
catch (...)
{
  planner_busy_.store(false);
  if (rclcpp::ok()) {
    RCLCPP_WARN(this->get_logger(), "Planner action thread exited after unknown exception");
  }
}

void PlannerNode::loadRacingLine()
{
  std::ifstream file(racing_line_file_);
  if (!file.is_open()) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to open racing line file: %s",
      racing_line_file_.c_str());
    return;
  }

  racing_line_.clear();
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    double x, y, v;
    char comma;
    if (ss >> x >> comma >> y >> comma >> v) {
      racing_line_.emplace_back(x, y, v);
    }
  }

  if (racing_line_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Racing line is empty");
    return;
  }

  planner_->setRacingLine(racing_line_);
}

LocalPlannerIntent PlannerNode::intentFromAction(uint8_t intent) const
{
  switch (intent) {
    case PlanPath::Goal::OVERTAKE:
      return LocalPlannerIntent::OVERTAKE;
    case PlanPath::Goal::MERGE:
      return LocalPlannerIntent::MERGE;
    case PlanPath::Goal::FOLLOW_RACING_LINE:
      return LocalPlannerIntent::FOLLOW_RACING_LINE;
    default:
      return default_intent_;
  }
}

void PlannerNode::publishPlannerViz(const LocalFrenetPlan & plan)
{
  if (!rclcpp::ok()) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = "map";
  clear.header.stamp = this->now();
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  visualization_msgs::msg::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.header.stamp = this->now();
  line_strip.ns = "selected_path";
  line_strip.id = 0;
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::msg::Marker::ADD;
  line_strip.scale.x = 0.08;
  line_strip.color.r = 0.0;
  line_strip.color.g = 1.0;
  line_strip.color.b = 0.15;
  line_strip.color.a = 1.0;

  for (const auto & p : plan.path) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = 0.0;
    line_strip.points.push_back(pt);
  }

  markers.markers.push_back(line_strip);
  viz_pub_->publish(markers);
}

local_planning::Odometry PlannerNode::rosToOdometry(
  const nav_msgs::msg::Odometry::SharedPtr & msg)
{
  local_planning::Odometry odom;
  odom.position.x = msg->pose.pose.position.x;
  odom.position.y = msg->pose.pose.position.y;
  odom.velocity = msg->twist.twist.linear.x;
  odom.heading = tf2::getYaw(msg->pose.pose.orientation);
  return odom;
}

local_planning::OccupancyGrid PlannerNode::rosToOccupancyGrid(
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

void PlannerNode::buildClearanceMasks(local_planning::OccupancyGrid & grid) const
{
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= 1e-6) {
    grid.definitely_blocked_mask.clear();
    grid.needs_exact_check_mask.clear();
    grid.has_clearance_cache = false;
    return;
  }

  const size_t cell_count = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
  grid.definitely_blocked_mask.assign(cell_count, 0);
  grid.needs_exact_check_mask.assign(cell_count, 0);

  const double cell_half_diagonal = 0.5 * std::sqrt(2.0) * grid.resolution;
  const double blocked_radius = std::max(
    0.0, planner_config_.collision_circle_radius_m - cell_half_diagonal);
  const double exact_check_radius = std::max(
    blocked_radius,
    planner_config_.collision_circle_radius_m +
    planner_config_.soft_inflation_distance_m + cell_half_diagonal);

  auto makeDiskOffsets = [&](double radius_m) {
      std::vector<std::pair<int, int>> offsets;
      const int max_cells = std::max(
        0, static_cast<int>(std::ceil(radius_m / grid.resolution)));
      const double radius_sq = radius_m * radius_m;
      for (int dr = -max_cells; dr <= max_cells; ++dr) {
        for (int dc = -max_cells; dc <= max_cells; ++dc) {
          const double dx = static_cast<double>(dc) * grid.resolution;
          const double dy = static_cast<double>(dr) * grid.resolution;
          if (dx * dx + dy * dy <= radius_sq) {
            offsets.emplace_back(dr, dc);
          }
        }
      }
      return offsets;
    };

  const std::vector<std::pair<int, int>> blocked_offsets = makeDiskOffsets(blocked_radius);
  const std::vector<std::pair<int, int>> exact_offsets = makeDiskOffsets(exact_check_radius);

  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const size_t source_index = static_cast<size_t>(row * grid.width + col);
      if (grid.data[source_index] < planner_config_.occupied_threshold) {
        continue;
      }

      for (const auto & offset : blocked_offsets) {
        const int masked_row = row + offset.first;
        const int masked_col = col + offset.second;
        if (masked_row < 0 || masked_row >= grid.height || masked_col < 0 || masked_col >= grid.width) {
          continue;
        }

        grid.definitely_blocked_mask[
          static_cast<size_t>(masked_row * grid.width + masked_col)] = 1;
      }

      for (const auto & offset : exact_offsets) {
        const int masked_row = row + offset.first;
        const int masked_col = col + offset.second;
        if (masked_row < 0 || masked_row >= grid.height || masked_col < 0 || masked_col >= grid.width) {
          continue;
        }

        grid.needs_exact_check_mask[
          static_cast<size_t>(masked_row * grid.width + masked_col)] = 1;
      }
    }
  }

  grid.has_clearance_cache = true;
}

nav_msgs::msg::Path PlannerNode::pathToRosPath(
  const std::vector<local_planning::Point> & path, const std::string & frame_id)
{
  nav_msgs::msg::Path ros_path;
  ros_path.header.frame_id = frame_id;
  ros_path.header.stamp = this->now();

  for (const auto & p : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = ros_path.header;
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.position.z = p.velocity;
    pose.pose.orientation.w = 1.0;
    ros_path.poses.push_back(pose);
  }
  return ros_path;
}

bool PlannerNode::transformPathToControllerFrame(
  const nav_msgs::msg::Path & planner_path, nav_msgs::msg::Path & controller_path)
{
  controller_path = planner_path;
  controller_path.header.frame_id = controller_path_frame_;

  if (planner_path.header.frame_id == controller_path_frame_) {
    for (auto & pose : controller_path.poses) {
      pose.header = controller_path.header;
    }
    return true;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      controller_path_frame_, planner_path.header.frame_id,
      rclcpp::Time(planner_path.header.stamp), tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & stamped_ex) {
    try {
      transform = tf_buffer_->lookupTransform(
        controller_path_frame_, planner_path.header.frame_id,
        tf2::TimePointZero, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & latest_ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Cannot transform local path from '%s' to '%s': %s; latest TF fallback also failed: %s",
        planner_path.header.frame_id.c_str(), controller_path_frame_.c_str(),
        stamped_ex.what(), latest_ex.what());
      return false;
    }
  }

  tf2::Transform planner_to_controller;
  tf2::fromMsg(transform.transform, planner_to_controller);

  for (size_t i = 0; i < planner_path.poses.size(); ++i) {
    const auto & planner_pose = planner_path.poses[i];
    auto & controller_pose = controller_path.poses[i];
    const double velocity = planner_pose.pose.position.z;

    const tf2::Vector3 planner_point(
      planner_pose.pose.position.x,
      planner_pose.pose.position.y,
      0.0);
    const tf2::Vector3 controller_point = planner_to_controller * planner_point;

    controller_pose.header = controller_path.header;
    controller_pose.pose.position.x = controller_point.x();
    controller_pose.pose.position.y = controller_point.y();
    controller_pose.pose.position.z = velocity;
    controller_pose.pose.orientation.x = 0.0;
    controller_pose.pose.orientation.y = 0.0;
    controller_pose.pose.orientation.z = 0.0;
    controller_pose.pose.orientation.w = 1.0;
  }

  return true;
}

} // namespace local_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planning::PlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
