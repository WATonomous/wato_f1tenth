#include "planning/planner/planner_node.hpp"

#include "planning/planner/collision_checker.hpp"
#include "planning/planner/local_frenet_lattice_planner.hpp"
#include "planning/ros_adapters.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

namespace local_planning
{
using namespace std::placeholders;
using SteadyClock = std::chrono::steady_clock;

namespace
{
double elapsedMs(SteadyClock::time_point start)
{
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
}
}

PlannerNode::PlannerNode()
: Node("local_frenet_lattice_planner_node")
{
  declare_parameter<std::string>("racing_line_topic", "/global_planner/path");
  declare_parameter<double>("planner_rate_hz", 100.0);
  declare_parameter<double>("horizon_m", 6.0);
  declare_parameter<double>("layer_spacing_m", 0.5);
  declare_parameter<double>("lane_spacing_m", 0.1);
  declare_parameter<double>("max_lateral_offset_m", 1.8);
  declare_parameter<double>("max_path_angle_deg", 50.0);
  declare_parameter<double>("sample_spacing_m", 0.1);
  declare_parameter<double>("max_runtime_ms", 9.0);
  declare_parameter<double>("collision_circle_radius_m", 0.20);
  declare_parameter<double>("front_collision_circle_offset_m", 0.26);
  declare_parameter<double>("soft_inflation_distance_m", 0.18);
  declare_parameter<double>("soft_inflation_cost", 100.0);
  declare_parameter<int>("occupied_threshold", 50);
  declare_parameter<double>("friction_coeff", 1.0);
  declare_parameter<double>("min_velocity_mps", 0.5);
  declare_parameter<double>("max_velocity_mps", 10.0);
  declare_parameter<double>("time_weight", 1.0);
  declare_parameter<double>("curvature_change_weight", 0.4);
  declare_parameter<double>("follow_d_weight", 0.20);
  declare_parameter<double>("overtake_d_weight", 0.02);
  declare_parameter<double>("merge_d_weight", 0.20);
  declare_parameter<double>("merge_terminal_d_weight", 0.0);
  declare_parameter<std::string>("planner_path_frame", "map");
  declare_parameter<std::string>("controller_path_frame", "base_link");
  declare_parameter<std::string>("debug_path_topic", "/local_path_map");
  declare_parameter<bool>("angle_smoothing_enabled", false);
  declare_parameter<bool>("velocity_smoothing_enabled", false);
  declare_parameter<double>("velocity_smoothing_max_accel_mps2", 2.5);
  declare_parameter<double>("velocity_smoothing_max_decel_mps2", 2.5);

  racing_line_topic_ = get_parameter("racing_line_topic").as_string();
  planner_path_frame_ = get_parameter("planner_path_frame").as_string();
  controller_path_frame_ = get_parameter("controller_path_frame").as_string();
  debug_path_topic_ = get_parameter("debug_path_topic").as_string();
  const double rate = std::max(0.1, get_parameter("planner_rate_hz").as_double());
  worker_period_ = std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<double>(1.0 / rate));

#define LOAD_DOUBLE(field) planner_config_.field = get_parameter(#field).as_double()
  LOAD_DOUBLE(horizon_m); LOAD_DOUBLE(layer_spacing_m); LOAD_DOUBLE(lane_spacing_m);
  LOAD_DOUBLE(max_lateral_offset_m); LOAD_DOUBLE(max_path_angle_deg); LOAD_DOUBLE(sample_spacing_m);
  LOAD_DOUBLE(max_runtime_ms); LOAD_DOUBLE(collision_circle_radius_m);
  LOAD_DOUBLE(front_collision_circle_offset_m); LOAD_DOUBLE(soft_inflation_distance_m);
  LOAD_DOUBLE(soft_inflation_cost); LOAD_DOUBLE(friction_coeff); LOAD_DOUBLE(min_velocity_mps);
  LOAD_DOUBLE(max_velocity_mps); LOAD_DOUBLE(time_weight); LOAD_DOUBLE(curvature_change_weight);
  LOAD_DOUBLE(follow_d_weight); LOAD_DOUBLE(overtake_d_weight); LOAD_DOUBLE(merge_d_weight);
  LOAD_DOUBLE(merge_terminal_d_weight); LOAD_DOUBLE(velocity_smoothing_max_accel_mps2);
  LOAD_DOUBLE(velocity_smoothing_max_decel_mps2);
#undef LOAD_DOUBLE
  planner_config_.occupied_threshold = get_parameter("occupied_threshold").as_int();
  planner_config_.angle_smoothing_enabled = get_parameter("angle_smoothing_enabled").as_bool();
  planner_config_.velocity_smoothing_enabled =
    get_parameter("velocity_smoothing_enabled").as_bool();

  planner_ = std::make_unique<LocalFrenetLatticePlanner>();
  planner_->setConfig(planner_config_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 1, std::bind(&PlannerNode::odometryCallback, this, _1));
  occupancy_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/occupancy_grid", 1, std::bind(&PlannerNode::occupancyGridCallback, this, _1));
  racing_line_sub_ = create_subscription<nav_msgs::msg::Path>(
    racing_line_topic_, rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&PlannerNode::racingLineCallback, this, _1));
  intent_sub_ = create_subscription<msg::PlannerIntent>(
    "/planner_intent", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&PlannerNode::intentCallback, this, _1));

  path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 10);
  debug_path_pub_ = create_publisher<nav_msgs::msg::Path>(debug_path_topic_, 10);
  viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/local_frenet_lattice_viz", 10);
  status_pub_ = create_publisher<msg::PlannerStatus>("/planner_status", 10);
  worker_ = std::thread(&PlannerNode::workerLoop, this);
}

PlannerNode::~PlannerNode()
{
  stop_worker_.store(true);
  sleep_cv_.notify_all();
  if (worker_.joinable()) {worker_.join();}
}

void PlannerNode::odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(input_mutex_); current_odom_ = std::move(msg);
}

void PlannerNode::occupancyGridCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  auto grid = std::make_shared<OccupancyGrid>(rosToOccupancyGrid(*msg));
  CollisionChecker(planner_config_).buildClearanceCache(*grid);
  std::lock_guard<std::mutex> lock(input_mutex_);
  current_grid_msg_ = std::move(msg); current_grid_ = std::move(grid);
}

void PlannerNode::racingLineCallback(nav_msgs::msg::Path::SharedPtr msg)
{
  auto reference = std::make_shared<std::vector<Point>>();
  if (!msg->poses.empty()) {*reference = rosPathToRacingLine(*msg);}
  std::lock_guard<std::mutex> lock(input_mutex_);
  current_reference_msg_ = std::move(msg); current_reference_ = std::move(reference);
}

void PlannerNode::intentCallback(msg::PlannerIntent::SharedPtr message)
{
  LocalPlannerIntent intent;
  switch (message->intent) {
    case msg::PlannerIntent::OVERTAKE: intent = LocalPlannerIntent::OVERTAKE; break;
    case msg::PlannerIntent::MERGE: intent = LocalPlannerIntent::MERGE; break;
    case msg::PlannerIntent::FOLLOW_RACING_LINE: intent = LocalPlannerIntent::FOLLOW_RACING_LINE;
      break;
    default: return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_); current_intent_ = intent; has_intent_ = true;
}

void PlannerNode::workerLoop()
{
  while (!stop_worker_.load()) {
    const auto start = SteadyClock::now();
    runAttempt(start);
    const auto finish = SteadyClock::now();
    const auto elapsed = finish - start;
    if (elapsed >= worker_period_) {
      overruns_ += static_cast<uint64_t>(elapsed / worker_period_);
      continue;
    }
    std::unique_lock<std::mutex> lock(sleep_mutex_);
    sleep_cv_.wait_until(lock, start + worker_period_, [this] {return stop_worker_.load();});
  }
}

void PlannerNode::runAttempt(SteadyClock::time_point start)
{
  nav_msgs::msg::Odometry::SharedPtr odom_msg;
  nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg;
  nav_msgs::msg::Path::SharedPtr reference_msg;
  std::shared_ptr<const OccupancyGrid> grid;
  std::shared_ptr<const std::vector<Point>> reference;
  LocalPlannerIntent intent;
  bool has_intent;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    odom_msg = current_odom_; grid_msg = current_grid_msg_; grid = current_grid_;
    reference_msg = current_reference_msg_; reference = current_reference_;
    intent = current_intent_; has_intent = has_intent_;
  }
  ++completed_attempts_;
  if (!odom_msg || !grid_msg || !grid || !reference_msg || !has_intent) {
    publishStatus(msg::PlannerStatus::MISSING_INPUT, elapsedMs(start), 0,
      odom_msg, grid_msg, reference_msg, intent); return;
  }
  if (!reference || reference->empty()) {
    publishStatus(msg::PlannerStatus::INVALID_REFERENCE, elapsedMs(start), 0,
      odom_msg, grid_msg, reference_msg, intent); return;
  }
  if (reference != planner_reference_guard_) {
    planner_->setRacingLine(*reference); planner_reference_guard_ = reference;
  }
  const auto deadline = start + std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<double, std::milli>(planner_config_.max_runtime_ms));
  LocalFrenetPlan plan = planner_->plan(rosToOdometry(*odom_msg), *grid, intent, deadline);
  if (plan.status == LocalFrenetPlan::Status::DEADLINE_EXCEEDED) {
    publishStatus(msg::PlannerStatus::DEADLINE_EXCEEDED, elapsedMs(start), 0,
      odom_msg, grid_msg, reference_msg, intent); return;
  }
  if (plan.status == LocalFrenetPlan::Status::INVALID_REFERENCE) {
    publishStatus(msg::PlannerStatus::INVALID_REFERENCE, elapsedMs(start), 0,
      odom_msg, grid_msg, reference_msg, intent); return;
  }
  if (plan.status != LocalFrenetPlan::Status::SUCCESS || plan.path.empty()) {
    publishStatus(msg::PlannerStatus::NO_PATH, elapsedMs(start), 0,
      odom_msg, grid_msg, reference_msg, intent); return;
  }
  nav_msgs::msg::Path planner_path = pathToRosPath(plan.path, reference_msg->header);
  nav_msgs::msg::Path controller_path;
  if (!transformPathToControllerFrame(planner_path, controller_path)) {
    publishStatus(msg::PlannerStatus::TF_FAILURE, elapsedMs(start), plan.path.size(),
      odom_msg, grid_msg, reference_msg, intent); return;
  }
  path_pub_->publish(controller_path); debug_path_pub_->publish(planner_path);
  publishPlannerViz(plan);
  last_trajectory_stamp_ = controller_path.header.stamp;
  publishStatus(msg::PlannerStatus::SUCCESS, elapsedMs(start), plan.path.size(),
    odom_msg, grid_msg, reference_msg, intent);
}

void PlannerNode::publishStatus(
  uint8_t status, double runtime_ms, uint32_t path_points,
  const nav_msgs::msg::Odometry::SharedPtr & odom,
  const nav_msgs::msg::OccupancyGrid::SharedPtr & grid,
  const nav_msgs::msg::Path::SharedPtr & reference, LocalPlannerIntent intent)
{
  msg::PlannerStatus out;
  out.header.stamp = now(); out.header.frame_id = planner_path_frame_;
  out.status = status; out.intent = static_cast<uint8_t>(intent);
  if (odom) {out.odometry_header = odom->header;}
  if (grid) {out.grid_header = grid->header;}
  if (reference) {out.reference_header = reference->header;}
  out.runtime_ms = runtime_ms; out.path_points = path_points;
  out.completed_attempts = completed_attempts_; out.overruns = overruns_;
  out.last_trajectory_stamp = last_trajectory_stamp_;
  status_pub_->publish(out);
}

nav_msgs::msg::Path PlannerNode::pathToRosPath(
  const std::vector<Point> & path, const std_msgs::msg::Header & source_header)
{
  nav_msgs::msg::Path result; result.header = source_header;
  result.header.frame_id = planner_path_frame_; result.header.stamp = now();
  result.poses.reserve(path.size());
  for (const auto & p : path) {
    geometry_msgs::msg::PoseStamped pose; pose.header = result.header;
    pose.pose.position.x = p.x; pose.pose.position.y = p.y; pose.pose.position.z = p.velocity;
    pose.pose.orientation.w = 1.0; result.poses.push_back(pose);
  }
  return result;
}

bool PlannerNode::transformPathToControllerFrame(
  const nav_msgs::msg::Path & planner_path, nav_msgs::msg::Path & controller_path)
{
  controller_path = planner_path; controller_path.header.frame_id = controller_path_frame_;
  if (planner_path.header.frame_id == controller_path_frame_) {return true;}
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(controller_path_frame_, planner_path.header.frame_id,
      tf2::TimePointZero, tf2::durationFromSec(0.001));
  } catch (const tf2::TransformException &) {
    return false;
  }
  tf2::Transform tf; tf2::fromMsg(transform.transform, tf);
  for (auto & pose : controller_path.poses) {
    const double velocity = pose.pose.position.z;
    const tf2::Vector3 point = tf * tf2::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0);
    pose.header = controller_path.header; pose.pose.position.x = point.x();
    pose.pose.position.y = point.y(); pose.pose.position.z = velocity;
    pose.pose.orientation.x = pose.pose.orientation.y = pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
  }
  return true;
}

void PlannerNode::publishPlannerViz(const LocalFrenetPlan & plan)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear; clear.header.frame_id = planner_path_frame_;
  clear.header.stamp = now(); clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  visualization_msgs::msg::Marker line; line.header = clear.header; line.ns = "selected_path";
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.08; line.color.g = 1.0; line.color.b = 0.15; line.color.a = 1.0;
  for (const auto & p : plan.path) {
    geometry_msgs::msg::Point point; point.x = p.x; point.y = p.y; line.points.push_back(point);
  }
  markers.markers.push_back(line); viz_pub_->publish(markers);
}

}  // namespace local_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<local_planning::PlannerNode>());
  rclcpp::shutdown(); return 0;
}
