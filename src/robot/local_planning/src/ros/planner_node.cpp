#include "local_planning/ros/planner_node.hpp"

#include "local_planning/ros/ros_adapters.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace local_planning
{

PlannerNode::PlannerNode()
: Node("planner_node"),
  config_(loadConfig()),
  curve_generator_(config_.curve),
  maneuver_builder_(reference_, curve_generator_, config_.maneuver),
  state_machine_(reference_, config_.state),
  planner_(reference_, maneuver_builder_, config_.planner),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  reference_.setProjectionConfig(config_.projection);
  const auto latched = rclcpp::QoS(1).transient_local().reliable();

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(config_.odom_topic, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {odom_ = std::move(msg);});
  grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(config_.occupancy_grid_topic, 1,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        grid_ = rosToOccupancyGrid(*msg);
        planner_.buildGridCache(grid_);
        has_grid_ = true;
      });
  racing_line_sub_ = create_subscription<nav_msgs::msg::Path>(config_.racing_line_topic, latched,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        if (!reference_.setRacingLine(rosPathToRacingLine(*msg))) {
          RCLCPP_ERROR(get_logger(), "Invalid racing line");
        }
      });
  steering_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      config_.steering_command_topic, 10,
    [this](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
      steering_angle_ = msg->drive.steering_angle;
      steering_received_ = std::chrono::steady_clock::now();
      has_steering_ = true;
      });

  local_path_pub_ = create_publisher<nav_msgs::msg::Path>(config_.local_path_topic, 10);
  local_path_map_pub_ = create_publisher<nav_msgs::msg::Path>(config_.local_path_map_topic, 10);
  overtake_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
    config_.overtake_ready_topic, latched);
  decision_pub_ = create_publisher<msg::PlannerDecision>(config_.decision_topic, 10);
  visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    config_.visualization_topic, 10);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / config_.planner_rate_hz),
    std::bind(&PlannerNode::planningCycle, this));

  if (std::abs(config_.state.compat_lateral_m -
    config_.maneuver.sideDeadbandM()) > 1e-6)
  {
    RCLCPP_WARN(get_logger(), "compat_lateral_m differs from side deadband");
  }
  if (config_.state.overtake_start_gap_m >= config_.maneuver.horizon_m) {
    RCLCPP_WARN(get_logger(), "overtake_start_gap_m must be below horizon_m");
  }
  if (config_.planner.max_velocity_mps > std::sqrt(
    2.0 * config_.planner.max_decel_mps2 * config_.maneuver.horizon_m))
  {
    RCLCPP_WARN(get_logger(), "max velocity exceeds horizon braking envelope");
  }
}

PlannerNode::NodeConfig PlannerNode::loadConfig()
{
  NodeConfig cfg;
  cfg.maneuver.horizon_m = declare_parameter("horizon_m", 6.0);
  const double radius = declare_parameter("collision_circle_radius_m", 0.20);
  cfg.maneuver.collision_circle_radius_m = radius;
  cfg.planner.collision_circle_radius_m = radius;
  cfg.maneuver.overtake_s_offsets_from_opponent_rear_m = declare_parameter(
    "overtake_s_offsets_from_opponent_rear_m", std::vector<double>{0.0, 0.5, 1.0});
  cfg.maneuver.passing_d_magnitudes_m = declare_parameter(
    "passing_d_magnitudes_m", std::vector<double>{0.55, 0.75});
  cfg.maneuver.overtake_heading_offsets_rad = declare_parameter(
    "overtake_heading_offsets_rad", std::vector<double>{-0.15, 0.0, 0.15});
  cfg.maneuver.overtake_curvature_multipliers = declare_parameter(
    "overtake_curvature_multipliers", std::vector<double>{0.0, 0.5, 1.0});
  cfg.maneuver.pass_transition_distances_m = declare_parameter(
    "pass_transition_distances_m", std::vector<double>{6.0, 3.0, 1.0});
  cfg.maneuver.merge_completion_distances_m = declare_parameter(
    "merge_completion_distances_m", std::vector<double>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0});

  cfg.state.corridor_half_width_m = declare_parameter("corridor_half_width_m", 0.25);
  cfg.state.overlap_gap_m = declare_parameter("overlap_gap_m", 0.80);
  cfg.state.clear_gap_m = declare_parameter("clear_gap_m", 1.50);
  cfg.state.overtake_start_gap_m = declare_parameter("overtake_start_gap_m", 3.00);
  cfg.state.compat_lateral_m = declare_parameter("compat_lateral_m", 0.40);
  cfg.state.compat_heading_rad = declare_parameter("compat_heading_rad", 0.15);

  cfg.planner.front_collision_circle_offset_m = declare_parameter(
    "front_collision_circle_offset_m", 0.26);
  cfg.planner.soft_inflation_distance_m = declare_parameter("soft_inflation_distance_m", 0.18);
  cfg.planner.occupied_threshold = declare_parameter("occupied_threshold", 50);
  cfg.planner.friction_coeff = declare_parameter("friction_coeff", 1.0);
  cfg.planner.min_velocity_mps = declare_parameter("min_velocity_mps", 0.0);
  cfg.planner.max_velocity_mps = declare_parameter("max_velocity_mps", 7.7);
  cfg.planner.max_accel_mps2 = declare_parameter("max_accel_mps2", 5.0);
  cfg.planner.max_decel_mps2 = declare_parameter("max_decel_mps2", 5.0);
  cfg.planner.overtake_speed_scale = declare_parameter("overtake_speed_scale", 1.1);
  cfg.planner.treat_out_of_grid_as_free = declare_parameter("treat_out_of_grid_as_free", false);

  cfg.curve.sample_spacing_m = declare_parameter("sample_spacing_m", 0.1);
  cfg.curve.max_curvature_inv_m = declare_parameter("max_curvature_inv_m", 1.74);
  cfg.curve.max_arc_length_m = declare_parameter("max_arc_length_m", 12.0);
  cfg.projection.seed_window_m = declare_parameter("seed_window_m", 3.0);
  cfg.projection.tangent_tolerance_rad = declare_parameter("tangent_tolerance_rad", 1.2);

  cfg.planner_rate_hz = declare_parameter("planner_rate_hz", 20.0);
  cfg.racing_line_topic = declare_parameter("racing_line_topic", "/global_planner/path");
  cfg.occupancy_grid_topic = declare_parameter("occupancy_grid_topic", "/occupancy_grid");
  cfg.odom_topic = declare_parameter("odom_topic", "/odom");
  cfg.steering_command_topic = declare_parameter("steering_command_topic", "/drive/autonomy");
  cfg.steering_command_timeout_s = declare_parameter("steering_command_timeout_s", 0.06);
  cfg.wheelbase_m = declare_parameter("wheelbase_m", 0.33);
  cfg.use_steering_start_curvature = declare_parameter("use_steering_start_curvature", true);
  cfg.map_frame = declare_parameter("map_frame", "map");
  cfg.controller_frame = declare_parameter("controller_frame", "base_link");
  cfg.local_path_topic = declare_parameter("local_path_topic", "/local_path");
  cfg.local_path_map_topic = declare_parameter("local_path_map_topic", "/local_path_map");
  cfg.overtake_ready_topic = declare_parameter("overtake_ready_topic", "/overtake_ready");
  cfg.decision_topic = declare_parameter("decision_topic", "/planner_decision");
  cfg.visualization_topic = declare_parameter("visualization_topic", "/local_planner_viz");
  return cfg;
}

void PlannerNode::planningCycle()
{
  if (!odom_ || !has_grid_ || !reference_.valid()) {
    publishDecision(PlannerDecisionData{});
    return;
  }

  Odometry odom = rosToOdometry(*odom_);
  const bool steering_fresh = has_steering_ &&
    std::chrono::duration<double>(std::chrono::steady_clock::now() -
    steering_received_).count() <= config_.steering_command_timeout_s;
  if (steering_fresh) {odom.steering_angle = steering_angle_;}

  state_machine_.update(odom, grid_);
  BoundaryState ego;
  ego.x = odom.position.x;
  ego.y = odom.position.y;
  ego.heading = odom.heading;
  ego.speed = odom.velocity;
  if (config_.use_steering_start_curvature && steering_fresh) {
    ego.curvature = std::tan(odom.steering_angle) / config_.wheelbase_m;
  }

  auto result = planner_.plan(state_machine_.state(), ego, grid_);
  result.decision.start_curvature_from_steering =
    config_.use_steering_start_curvature && steering_fresh;
  publishDecision(result.decision);

  if (result.decision.requested_intent == PlannerIntent::FOLLOW_RACING_LINE) {
    publishOvertakeReady(false);
    return;
  }
  if (result.decision.executed_mode == ExecutedMode::BRAKING_UNAVAILABLE) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Braking path unavailable");
    return;
  }
  if (result.selected_index < 0) {return;}

  const auto map_path = pathMessage(
    result.pool.at(static_cast<std::size_t>(result.selected_index)).path);
  nav_msgs::msg::Path controller_path;
  if (!transformPathToControllerFrame(map_path, controller_path)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Local path transform unavailable");
    return;
  }
  local_path_map_pub_->publish(map_path);
  local_path_pub_->publish(controller_path);
  publishMarkers(result);
  publishOvertakeReady(true);
}

nav_msgs::msg::Path PlannerNode::pathMessage(const Path & path) const
{
  nav_msgs::msg::Path msg;
  msg.header.stamp = now();
  msg.header.frame_id = config_.map_frame;
  msg.poses.reserve(path.size());
  for (const auto & sample : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = sample.x;
    pose.pose.position.y = sample.y;
    pose.pose.position.z = sample.speed;
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }
  return msg;
}

bool PlannerNode::transformPathToControllerFrame(
  const nav_msgs::msg::Path & map_path,
  nav_msgs::msg::Path & controller_path)
{
  controller_path = map_path;
  controller_path.header.frame_id = config_.controller_frame;
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(config_.controller_frame, map_path.header.frame_id,
        tf2::TimePointZero, tf2::durationFromSec(0.001));
  } catch (const tf2::TransformException &) {
    return false;
  }
  tf2::Transform tf;
  tf2::fromMsg(transform.transform, tf);
  for (auto & pose : controller_path.poses) {
    const double speed = pose.pose.position.z;
    const tf2::Vector3 point = tf * tf2::Vector3(
      pose.pose.position.x, pose.pose.position.y, 0.0);
    pose.header = controller_path.header;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = speed;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
  }
  return true;
}

void PlannerNode::publishDecision(const PlannerDecisionData & data)
{
  msg::PlannerDecision out;
  out.header.stamp = now();
  out.header.frame_id = config_.map_frame;
  out.requested_intent = static_cast<uint8_t>(data.requested_intent);
  out.relative_position = static_cast<uint8_t>(data.relative_position);
  out.opponent_detected = data.opponent_detected;
  out.opponent_gap_m = data.opponent_gap_m;
  out.executed_mode = static_cast<uint8_t>(data.executed_mode);
  out.candidate_source = static_cast<uint8_t>(data.candidate_source);
  out.projection_seed_was_stale = data.projection_seed_was_stale;
  out.clearance_class = static_cast<uint8_t>(data.clearance_class);
  out.minimum_clearance_m = data.minimum_clearance_m;
  out.max_abs_curvature_inv_m = data.max_abs_curvature_inv_m;
  out.min_speed_mps = data.min_speed_mps;
  out.max_speed_mps = data.max_speed_mps;
  out.start_curvature_inv_m = data.start_curvature_inv_m;
  out.start_curvature_from_steering = data.start_curvature_from_steering;
  out.terminal_d_m = data.terminal_d_m;
  out.best_cost_s = data.best_cost_s;
  out.median_cost_s = data.median_cost_s;
  out.generated_count = data.generated_count;
  out.collision_rejected = data.collision_rejected;
  out.out_of_grid_rejected = data.out_of_grid_rejected;
  out.velocity_rejected = data.velocity_rejected;
  out.valid_candidate_count = data.valid_candidate_count;
  out.cycle_time_ms = data.cycle_time_ms;
  decision_pub_->publish(out);
}

void PlannerNode::publishOvertakeReady(bool ready)
{
  if (last_overtake_ready_ && *last_overtake_ready_ == ready) {return;}
  std_msgs::msg::Bool msg;
  msg.data = ready;
  overtake_ready_pub_->publish(msg);
  last_overtake_ready_ = ready;
}

void PlannerNode::publishMarkers(const LocalPlanResult & result)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header.stamp = now();
  clear.header.frame_id = config_.map_frame;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  for (std::size_t i = 0; i < result.pool.size(); ++i) {
    visualization_msgs::msg::Marker line;
    line.header = clear.header;
    line.ns = "candidate_paths";
    line.id = static_cast<int>(i);
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    const bool selected = static_cast<int>(i) == result.selected_index;
    line.scale.x = selected ? 0.08 : 0.025;
    line.color.a = selected ? 1.0F : 0.35F;
    line.color.g = selected ? 1.0F : 0.55F;
    line.color.b = selected ? 0.1F : 0.9F;
    for (const auto & sample : result.pool[i].path) {
      geometry_msgs::msg::Point point;
      point.x = sample.x;
      point.y = sample.y;
      line.points.push_back(point);
    }
    markers.markers.push_back(std::move(line));
  }
  visualization_pub_->publish(markers);
}

}  // namespace local_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<local_planning::PlannerNode>());
  } catch (const std::invalid_argument & error) {
    RCLCPP_FATAL(rclcpp::get_logger("planner_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
