#include "local_planning/ros/planner_node.hpp"

#include "local_planning/ros/ros_adapters.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace local_planning
{
namespace
{
constexpr int kRareErrorThrottleMs = 10000;
}  // namespace

PlannerNode::PlannerNode()
: Node("planner_node"),
  config_(loadConfig()),
  curve_generator_(config_.curve),
  maneuver_builder_(
    reference_, curve_generator_, config_.maneuver, config_.vehicle_geometry),
  state_machine_(
    reference_, config_.state, config_.vehicle_geometry, config_.grid_policy),
  planner_(
    reference_, maneuver_builder_, config_.vehicle_geometry, config_.grid_policy,
    config_.collision, config_.velocity),
  diagnostics_(
    get_logger(), get_clock(),
    PlannerDiagnosticsConfig{
    config_.profiling_enabled,
    config_.profiling_log_every_n_cycles,
    config_.diagnostics_enabled,
    config_.profiling_intent_filter,
    config_.vehicle_geometry.fullWidthM(),
    config_.state.compat_heading_rad}),
  visualization_(
    reference_,
    PlannerVisualizationConfig{
    config_.map_frame,
    config_.publish_projection_markers,
    config_.vehicle_geometry.fullWidthM(),
    config_.state.compat_heading_rad}),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  reference_.setProjectionConfig(config_.projection);
  steering_received_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  const auto latched = rclcpp::QoS(1).transient_local().reliable();

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(config_.odom_topic, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {odom_ = std::move(msg);});
  grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(config_.occupancy_grid_topic, 1,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        const auto profile_started = std::chrono::steady_clock::now();
        grid_ = rosToOccupancyGrid(*msg);
        if (grid_.resolution > 0.0) {
          curve_generator_.setSampleSpacingM(grid_.resolution);
        }
        planner_.buildGridCache(grid_);
        has_grid_ = true;
        diagnostics_.recordGridUpdate(
          std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - profile_started).count(),
          grid_);
      });
  reference_track_sub_ = create_subscription<global_planner::msg::ReferenceTrack>(
    config_.reference_track_topic, latched,
    [this](const global_planner::msg::ReferenceTrack::SharedPtr msg) {
      if (msg->path.poses.size() != msg->widths.size()) {
        RCLCPP_ERROR(
          get_logger(), "Reference/width count mismatch: path=%zu widths=%zu",
          msg->path.poses.size(), msg->widths.size());
        reference_.clearTrackWidths();
        return;
      }
      std::vector<Point> points;
      points.reserve(msg->path.poses.size());
      for (const auto & pose : msg->path.poses) {
        points.emplace_back(
          pose.pose.position.x,
          pose.pose.position.y,
          pose.pose.position.z);
      }
      if (!reference_.setRacingLine(points)) {
        RCLCPP_ERROR(get_logger(), "Invalid driving reference");
        return;
      }
      std::vector<TrackWidth> widths;
      widths.reserve(msg->widths.size());
      for (const auto & width : msg->widths) {
        widths.push_back({width.right_m, width.left_m});
      }
      if (!reference_.setTrackWidths(widths, config_.width_lookup_spacing_m))
      {
        RCLCPP_ERROR(
          get_logger(),
          "Invalid reference widths (%zu widths for %zu spline waypoints); "
          "local maneuvers disabled",
          widths.size(), reference_.waypointCount());
        return;
      }
      RCLCPP_INFO(
        get_logger(), "Reference track ready: %zu waypoints, %zu width samples",
        reference_.waypointCount(), reference_.widthSampleCount());
      visualization_.publishTrackBounds(now(), *track_bounds_visualization_pub_);
    });
  steering_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      config_.steering_command_topic, 10,
    [this](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
      steering_angle_ = msg->drive.steering_angle;
      steering_received_ = now();
      has_steering_ = true;
      });

  local_path_pub_ = create_publisher<nav_msgs::msg::Path>(config_.local_path_topic, 10);
  local_path_map_pub_ = create_publisher<nav_msgs::msg::Path>(config_.local_path_map_topic, 10);
  overtake_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
    config_.overtake_ready_topic, latched);
  decision_pub_ = create_publisher<msg::PlannerDecision>(config_.decision_topic, 10);
  visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    config_.visualization_topic, 10);
  track_bounds_visualization_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(
    config_.track_bounds_visualization_topic, latched);
  projection_visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    config_.projection_visualization_topic, 10);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / config_.planner_rate_hz),
    std::bind(&PlannerNode::planningCycle, this));

  if (config_.state.overtake_start_gap_m >= config_.maneuver.horizon_m) {
    RCLCPP_WARN(get_logger(), "overtake_start_gap_m must be below horizon_m");
  }
  if (config_.velocity.max_velocity_mps > std::sqrt(
    2.0 * config_.velocity.max_decel_mps2 * config_.maneuver.horizon_m))
  {
    RCLCPP_WARN(get_logger(), "max velocity exceeds horizon braking envelope");
  }
}

PlannerNode::NodeConfig PlannerNode::loadConfig()
{
  NodeConfig cfg;
  cfg.maneuver.horizon_m = declare_parameter("horizon_m", 6.0);
  cfg.vehicle_geometry.collision_radius_m = declare_parameter(
    "collision_circle_radius_m", 0.20);
  cfg.maneuver.overtake_s_offsets_from_opponent_rear_m = declare_parameter(
    "overtake_s_offsets_from_opponent_rear_m", std::vector<double>{0.0, 0.5});
  cfg.maneuver.passing_d_magnitudes_m = declare_parameter(
    "passing_d_magnitudes_m", std::vector<double>{0.55, 0.75});
  cfg.maneuver.overtake_heading_offsets_rad = declare_parameter(
    "overtake_heading_offsets_rad", std::vector<double>{-0.15, 0.0, 0.15});
  cfg.maneuver.pass_transition_distances_m = declare_parameter(
    "pass_transition_distances_m", std::vector<double>{6.0, 3.0, 1.0});
  cfg.maneuver.merge_completion_distances_m = declare_parameter(
    "merge_completion_distances_m", std::vector<double>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0});

  cfg.state.corridor_half_width_m = declare_parameter("corridor_half_width_m", 0.25);
  cfg.state.overlap_gap_m = declare_parameter("overlap_gap_m", 0.80);
  cfg.state.clear_gap_m = declare_parameter("clear_gap_m", 1.00);
  cfg.state.overtake_start_gap_m = declare_parameter("overtake_start_gap_m", 3.00);
  cfg.state.compat_heading_rad = declare_parameter("compat_heading_rad", 1.05);

  cfg.vehicle_geometry.front_circle_offset_m = declare_parameter(
    "front_collision_circle_offset_m", 0.26);
  cfg.collision.soft_inflation_distance_m = declare_parameter(
    "soft_inflation_distance_m", 0.18);
  cfg.grid_policy.occupied_threshold = declare_parameter("occupied_threshold", 50);
  cfg.grid_policy.treat_unknown_as_free = declare_parameter("treat_unknown_as_free", true);
  cfg.grid_policy.treat_out_of_grid_as_free = declare_parameter(
    "treat_out_of_grid_as_free", false);
  cfg.velocity.friction_coeff = declare_parameter("friction_coeff", 1.0);
  cfg.velocity.min_velocity_mps = declare_parameter("min_velocity_mps", 0.0);
  cfg.velocity.max_velocity_mps = declare_parameter("max_velocity_mps", 7.7);
  cfg.velocity.max_accel_mps2 = declare_parameter("max_accel_mps2", 5.0);
  cfg.velocity.max_decel_mps2 = declare_parameter("max_decel_mps2", 5.0);
  cfg.velocity.overtake_speed_scale = declare_parameter("overtake_speed_scale", 1.1);

  cfg.curve.sample_spacing_m = declare_parameter("sample_spacing_m", 0.1);
  cfg.curve.max_curvature_inv_m = declare_parameter("max_curvature_inv_m", 1.74);
  cfg.curve.max_path_angle_deg = declare_parameter("max_path_angle_deg", 60.0);
  cfg.projection.seed_window_m = declare_parameter("seed_window_m", 2.0);
  cfg.projection.tangent_tolerance_rad = declare_parameter("tangent_tolerance_rad", 1.2);
  cfg.projection.max_plausible_offset_m = declare_parameter("max_plausible_offset_m", 3.0);

  cfg.planner_rate_hz = declare_parameter("planner_rate_hz", 20.0);
  cfg.reference_track_topic = declare_parameter(
    "reference_track_topic", "/global_planner/reference_track");
  cfg.width_lookup_spacing_m = declare_parameter("width_lookup_spacing_m", 0.10);
  cfg.occupancy_grid_topic = declare_parameter("occupancy_grid_topic", "/occupancy_grid");
  cfg.odom_topic = declare_parameter("odom_topic", "/odom");
  cfg.steering_command_topic = declare_parameter("steering_command_topic", "/drive/autonomy");
  cfg.steering_command_timeout_s = declare_parameter("steering_command_timeout_s", 0.06);
  cfg.odom_timeout_s = declare_parameter("odom_timeout_s", 0.25);
  cfg.wheelbase_m = declare_parameter("wheelbase_m", 0.33);
  cfg.use_steering_start_curvature = declare_parameter("use_steering_start_curvature", true);
  cfg.profiling_enabled = declare_parameter("profiling_enabled", true);
  cfg.profiling_log_every_n_cycles = declare_parameter("profiling_log_every_n_cycles", 20);
  cfg.diagnostics_enabled = declare_parameter("diagnostics_enabled", true);
  const std::string intent_filter = declare_parameter("profiling_intent_filter", std::string());
  if (!intent_filter.empty()) {
    for (const PlannerIntent intent : {PlannerIntent::FOLLOW_RACING_LINE, PlannerIntent::OVERTAKE,
        PlannerIntent::PASS, PlannerIntent::MERGE})
    {
      if (intentToString(intent) == intent_filter) {
        cfg.profiling_intent_filter = intent;
      }
    }
    if (!cfg.profiling_intent_filter) {
      RCLCPP_WARN(
        get_logger(), "Unknown profiling_intent_filter '%s'; profiling every intent",
        intent_filter.c_str());
    }
  }
  cfg.map_frame = declare_parameter("map_frame", "map");
  cfg.controller_frame = declare_parameter("controller_frame", "base_link");
  cfg.local_path_topic = declare_parameter("local_path_topic", "/local_path");
  cfg.local_path_map_topic = declare_parameter("local_path_map_topic", "/local_path_map");
  cfg.overtake_ready_topic = declare_parameter("overtake_ready_topic", "/overtake_ready");
  cfg.decision_topic = declare_parameter("decision_topic", "/planner_decision");
  cfg.visualization_topic = declare_parameter("visualization_topic", "/local_planner_viz");
  cfg.track_bounds_visualization_topic = declare_parameter(
    "track_bounds_visualization_topic", "/local_planner_track_bounds_viz");
  cfg.projection_visualization_topic = declare_parameter(
    "projection_visualization_topic", "/local_planner_projection_viz");
  cfg.publish_projection_markers = declare_parameter("publish_projection_markers", true);
  return cfg;
}

void PlannerNode::planningCycle()
{
  const auto cycle_started = std::chrono::steady_clock::now();
  CycleProfile profile;
  const auto finishProfile = [&]() {
      profile.ros.cycle_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - cycle_started).count();
      diagnostics_.recordCycle(std::move(profile));
    };
  if (!odom_ || !has_grid_ || !reference_.valid()) {
    publishDecision(PlannerDecisionData{});
    finishProfile();
    return;
  }

  const auto odom_started = std::chrono::steady_clock::now();
  const auto odom_in_map = odometryInMap();
  profile.ros.odom_conversion_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - odom_started).count();
  if (!odom_in_map) {
    publishDecision(PlannerDecisionData{});
    finishProfile();
    return;
  }
  profile.outcome.inputs_ready = true;
  Odometry odom = *odom_in_map;
  profile.outcome.steering_fresh = has_steering_ &&
    std::abs((now() - steering_received_).seconds()) <= config_.steering_command_timeout_s;
  if (profile.outcome.steering_fresh) {odom.steering_angle = steering_angle_;}

  const auto state_started = std::chrono::steady_clock::now();
  state_machine_.update(odom, grid_);
  profile.ros.state_update_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - state_started).count();
  const auto projection_marker_started = std::chrono::steady_clock::now();
  visualization_.publishProjection(
    odom, state_machine_.state(), now(), *projection_visualization_pub_);
  profile.ros.marker_publish_ms += std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - projection_marker_started).count();
  BoundaryState ego;
  ego.x = odom.position.x;
  ego.y = odom.position.y;
  ego.heading = odom.heading;
  ego.speed = odom.velocity;
  if (config_.use_steering_start_curvature && profile.outcome.steering_fresh) {
    ego.curvature = std::tan(odom.steering_angle) / config_.wheelbase_m;
  }

  if (grid_.resolution > 0.0) {
    curve_generator_.setSampleSpacingM(grid_.resolution);
  }

  const auto planner_started = std::chrono::steady_clock::now();
  auto result = planner_.plan(state_machine_.state(), ego, grid_);
  profile.ros.planner_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - planner_started).count();
  profile.core = result.profile;
  if (result.decision.requested_intent == PlannerIntent::MERGE) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "MERGE_CHECK requested=1 bounds_ready=%d generated=%u valid=%u "
      "collision_rej=%u out_of_grid_rej=%u track_rej=%u velocity_rej=%u "
      "selected=%d executed_mode=%d path_samples=%u collision_poses=%u "
      "ego_s=%.2f ego_d=%.2f terminal_d=%.2f clearance=%d min_clearance=%.2f",
      result.decision.track_bounds_ready ? 1 : 0,
      result.decision.generated_count,
      result.decision.valid_candidate_count,
      result.decision.collision_rejected,
      result.decision.out_of_grid_rejected,
      result.decision.track_bounds_rejected,
      result.decision.velocity_rejected,
      result.selected_index,
      static_cast<int>(result.decision.executed_mode),
      result.profile.total_path_samples,
      result.profile.collision_poses_checked,
      result.decision.ego_s_m,
      result.decision.ego_d_m,
      result.decision.terminal_d_m,
      static_cast<int>(result.decision.clearance_class),
      result.decision.minimum_clearance_m);
  }
  result.decision.start_curvature_from_steering =
    config_.use_steering_start_curvature && profile.outcome.steering_fresh;
  profile.outcome.decision = result.decision;
  const auto decision_publish_started = std::chrono::steady_clock::now();
  publishDecision(result.decision);
  profile.ros.decision_publish_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - decision_publish_started).count();

  if (result.decision.requested_intent == PlannerIntent::FOLLOW_RACING_LINE) {
    const auto marker_publish_started = std::chrono::steady_clock::now();
    publishOvertakeReady(false);
    profile.ros.marker_publish_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - marker_publish_started).count();
    finishProfile();
    return;
  }
  if (result.decision.executed_mode == ExecutedMode::BRAKING_UNAVAILABLE) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kRareErrorThrottleMs, "Braking path unavailable");
    publishOvertakeReady(false);
    finishProfile();
    return;
  }
  if (result.selected_index < 0) {
    publishOvertakeReady(false);
    finishProfile();
    return;
  }

  const auto path_message_started = std::chrono::steady_clock::now();
  const auto map_path = pathToRos(
    result.pool.at(static_cast<std::size_t>(result.selected_index)).path,
    now(), config_.map_frame);
  profile.ros.path_message_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - path_message_started).count();
  // The controller contract is the map-frame path. Pure pursuit re-transforms
  // the lookahead point every tick, so a missing map→base_link TF must not
  // starve /local_path_map or clear /overtake_ready. /local_path is leftover
  // viz/compat and is only published when that transform is available.
  const auto path_publish_started = std::chrono::steady_clock::now();
  local_path_map_pub_->publish(map_path);
  profile.outcome.path_published = true;
  nav_msgs::msg::Path controller_path;
  const auto tf_started = std::chrono::steady_clock::now();
  if (transformPathToControllerFrame(map_path, controller_path)) {
    local_path_pub_->publish(controller_path);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Local path transform unavailable");
  }
  profile.ros.tf_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - tf_started).count();
  profile.ros.path_publish_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - path_publish_started).count();
  const auto marker_publish_started = std::chrono::steady_clock::now();
  visualization_.publishCandidates(result, now(), *visualization_pub_);
  publishOvertakeReady(true);
  profile.ros.marker_publish_ms += std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - marker_publish_started).count();
  finishProfile();
}

std::optional<Odometry> PlannerNode::odometryInMap()
{
  if (!odom_) {
    return std::nullopt;
  }
  const rclcpp::Time stamp(odom_->header.stamp, now().get_clock_type());
  if (std::abs((now() - stamp).seconds()) > config_.odom_timeout_s) {
    return std::nullopt;
  }
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      config_.map_frame, config_.controller_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Ego pose transform unavailable");
    return std::nullopt;
  }
  Odometry odom;
  odom.position.x = transform.transform.translation.x;
  odom.position.y = transform.transform.translation.y;
  odom.velocity = odom_->twist.twist.linear.x;
  odom.heading = tf2::getYaw(transform.transform.rotation);
  return odom;
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
  decision_pub_->publish(plannerDecisionToRos(data, now(), config_.map_frame));
}

void PlannerNode::publishOvertakeReady(bool ready)
{
  if (last_overtake_ready_ && *last_overtake_ready_ == ready) {return;}
  std_msgs::msg::Bool msg;
  msg.data = ready;
  overtake_ready_pub_->publish(msg);
  last_overtake_ready_ = ready;
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
