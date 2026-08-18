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

void markHeldPathExecution(
  LocalPlanResult & result,
  const CollisionCheckResult & validation,
  PlannerIntent held_executed_intent)
{
  result.selected_index = -1;
  result.decision.executed_mode = ExecutedMode::HELD_PATH;
  result.decision.candidate_source = CandidateSource::NONE;
  result.decision.executed_intent = held_executed_intent;
  result.decision.selected_offset_tail = false;
  result.decision.selected_max_abs_d_m = 0.0;
  result.decision.clearance_class = validation.status;
  result.decision.minimum_clearance_m = validation.minimum_clearance_m;
  result.decision.max_abs_curvature_inv_m = 0.0;
  result.decision.min_speed_mps = 0.0;
  result.decision.max_speed_mps = 0.0;
  result.decision.terminal_d_m = 0.0;
  result.decision.braking_effort = 0.0;
  result.decision.braking_lookahead_m = 0.0;
}
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
    config_.collision, config_.velocity, config_.braking),
  diagnostics_(
    get_logger(), get_clock(),
    PlannerDiagnosticsConfig{
    config_.profiling_enabled,
    config_.profiling_log_every_n_cycles,
    config_.diagnostics_enabled,
    config_.profiling_intent_filter,
    config_.state.follow_exit_abs_d_m,
    config_.state.compat_heading_rad}),
  visualization_(
    reference_,
    PlannerVisualizationConfig{
    config_.map_frame,
    config_.publish_projection_markers,
    config_.state.follow_exit_abs_d_m,
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
        ++costmap_sequence_;
        costmap_stamp_s_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()).seconds();
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
  if (config_.publish_all_candidates) {
    all_candidates_visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      config_.all_candidates_visualization_topic, 10);
    RCLCPP_INFO(
      get_logger(), "Publishing every generated candidate on %s (debug; costs generation)",
      config_.all_candidates_visualization_topic.c_str());
  }

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / config_.planner_rate_hz),
    std::bind(&PlannerNode::planningCycle, this));

  if (config_.state.engagement_enter_gap_m >= config_.maneuver.horizon_m) {
    RCLCPP_WARN(get_logger(), "engagement_enter_gap_m must be below horizon_m");
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
  cfg.maneuver.horizon_m = declare_parameter("horizon_m", 4.0);
  cfg.vehicle_geometry.collision_radius_m = declare_parameter(
    "collision_circle_radius_m", 0.20);
  cfg.maneuver.overtake_s_offsets_from_opponent_rear_m = declare_parameter(
    "overtake_s_offsets_from_opponent_rear_m", std::vector<double>{0.0, 0.5});
  cfg.maneuver.passing_d_magnitudes_m = declare_parameter(
    "passing_d_magnitudes_m", std::vector<double>{0.55, 0.75});
  cfg.maneuver.overtake_heading_offsets_rad = declare_parameter(
    "overtake_heading_offsets_rad", std::vector<double>{-0.15, 0.0, 0.15});
  cfg.maneuver.pass_transition_distances_m = declare_parameter(
    "pass_transition_distances_m", std::vector<double>{4.0, 3.0, 1.0});
  cfg.maneuver.merge_completion_distances_m = declare_parameter(
    "merge_completion_distances_m", std::vector<double>{0.5, 1.0, 2.0, 3.0, 4.0});

  cfg.state.corridor_half_width_m = declare_parameter("corridor_half_width_m", 0.25);
  cfg.state.pass_enter_gap_m = declare_parameter("pass_enter_gap_m", 0.65);
  cfg.state.pass_exit_gap_m = declare_parameter("pass_exit_gap_m", 0.95);
  cfg.state.merge_enter_gap_m = declare_parameter("merge_enter_gap_m", -1.20);
  cfg.state.merge_exit_gap_m = declare_parameter("merge_exit_gap_m", -0.80);
  cfg.state.engagement_enter_gap_m = declare_parameter("engagement_enter_gap_m", 2.00);
  cfg.state.engagement_exit_gap_m = declare_parameter("engagement_exit_gap_m", 2.50);
  cfg.state.follow_enter_abs_d_m = declare_parameter("follow_enter_abs_d_m", 0.14);
  cfg.state.follow_exit_abs_d_m = declare_parameter("follow_exit_abs_d_m", 0.28);
  cfg.state.fast_confirmation_s = declare_parameter("fast_transition_confirmation_s", 0.05);
  cfg.state.slow_confirmation_s = declare_parameter("slow_transition_confirmation_s", 0.15);
  const int opponent_confirmation_grids = declare_parameter("opponent_confirmation_grids", 2);
  const int pass_merge_confirmation_grids = declare_parameter(
    "pass_merge_confirmation_grids", 3);
  const int merge_pass_confirmation_grids = declare_parameter(
    "merge_pass_confirmation_grids", 3);
  const int merge_probe_confirmation_cycles = declare_parameter(
    "merge_probe_confirmation_cycles", 3);
  cfg.state.compat_heading_rad = declare_parameter("compat_heading_rad", 1.05);
  if (cfg.state.follow_enter_abs_d_m > cfg.state.follow_exit_abs_d_m ||
    cfg.state.pass_enter_gap_m > cfg.state.pass_exit_gap_m ||
    cfg.state.merge_enter_gap_m > cfg.state.merge_exit_gap_m ||
    cfg.state.engagement_enter_gap_m > cfg.state.engagement_exit_gap_m ||
    cfg.state.fast_confirmation_s < 0.0 || cfg.state.slow_confirmation_s < 0.0 ||
    opponent_confirmation_grids < 1 || pass_merge_confirmation_grids < 1 ||
    merge_pass_confirmation_grids < 1 || merge_probe_confirmation_cycles < 1)
  {
    throw std::invalid_argument("invalid tactical hysteresis or confirmation configuration");
  }
  cfg.state.opponent_confirmation_grids = static_cast<uint32_t>(opponent_confirmation_grids);
  cfg.state.pass_merge_confirmation_grids =
    static_cast<uint32_t>(pass_merge_confirmation_grids);
  cfg.state.merge_pass_confirmation_grids =
    static_cast<uint32_t>(merge_pass_confirmation_grids);
  cfg.state.merge_probe_confirmation_cycles =
    static_cast<uint32_t>(merge_probe_confirmation_cycles);

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
  cfg.path_min_hold_s = declare_parameter("path_min_hold_s", 0.10);
  cfg.path_max_hold_s = declare_parameter("path_max_hold_s", 0.15);
  cfg.reference_track_topic = declare_parameter(
    "reference_track_topic", "/global_planner/reference_track");
  cfg.width_lookup_spacing_m = declare_parameter("width_lookup_spacing_m", 0.10);
  cfg.occupancy_grid_topic = declare_parameter("occupancy_grid_topic", "/occupancy_grid");
  cfg.odom_topic = declare_parameter("odom_topic", "/odom");
  cfg.steering_command_topic = declare_parameter("steering_command_topic", "/drive/autonomy");
  cfg.steering_command_timeout_s = declare_parameter("steering_command_timeout_s", 0.06);
  cfg.odom_timeout_s = declare_parameter("odom_timeout_s", 0.25);
  cfg.wheelbase_m = declare_parameter("wheelbase_m", 0.33);
  cfg.max_steering_angle_rad = declare_parameter("max_steering_angle_rad", 0.52);
  // Braking shares the horizon, sampling, friction and geometry the rest of the
  // planner runs on; only what is genuinely its own is declared separately.
  cfg.braking.horizon_m = cfg.maneuver.horizon_m;
  cfg.braking.sample_spacing_m = cfg.curve.sample_spacing_m;
  cfg.braking.friction_coeff = cfg.velocity.friction_coeff;
  cfg.braking.wheelbase_m = cfg.wheelbase_m;
  cfg.braking.max_steering_angle_rad = cfg.max_steering_angle_rad;
  cfg.braking.decel_mps2 = declare_parameter("braking_decel_mps2", 5.0);
  cfg.braking.min_velocity_mps = declare_parameter("braking_min_velocity_mps", 1.0);
  cfg.braking.pursuit_lookaheads_m = declare_parameter(
    "braking_pursuit_lookaheads_m", std::vector<double>{1.0, 2.0, 3.0});
  cfg.braking.effort_levels = declare_parameter(
    "braking_effort_levels", std::vector<double>{1.0, 0.0});
  cfg.use_steering_start_curvature = declare_parameter("use_steering_start_curvature", true);
  cfg.profiling_enabled = declare_parameter("profiling_enabled", true);
  cfg.profiling_log_every_n_cycles = declare_parameter("profiling_log_every_n_cycles", 20);
  cfg.diagnostics_enabled = declare_parameter("diagnostics_enabled", true);
  const auto intent_filter = declare_parameter(
    "profiling_intent_filter", std::vector<std::string>{});
  for (const std::string & name : intent_filter) {
    bool matched = false;
    for (const PlannerIntent intent : {PlannerIntent::FOLLOW_RACING_LINE, PlannerIntent::OVERTAKE,
        PlannerIntent::PASS, PlannerIntent::MERGE})
    {
      if (intentToString(intent) == name) {
        cfg.profiling_intent_filter.push_back(intent);
        matched = true;
      }
    }
    if (!matched) {
      RCLCPP_WARN(get_logger(), "Unknown profiling_intent_filter entry '%s'; ignored", name.c_str());
    }
  }
  // Every entry unknown is the same mistake as a typo'd single name: report
  // every intent rather than silently profiling nothing.
  if (!intent_filter.empty() && cfg.profiling_intent_filter.empty()) {
    RCLCPP_WARN(get_logger(), "No valid profiling_intent_filter entries; profiling every intent");
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
  cfg.publish_all_candidates = declare_parameter("publish_all_candidates", false);
  cfg.all_candidates_visualization_topic = declare_parameter(
    "all_candidates_visualization_topic", "/local_planner_all_candidates_viz");
  return cfg;
}

void PlannerNode::planningCycle()
{
  const auto cycle_started = std::chrono::steady_clock::now();
  const rclcpp::Time cycle_stamp = now();
  CycleProfile profile;
  const auto finishProfile = [&]() {
      profile.ros.cycle_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - cycle_started).count();
      diagnostics_.recordCycle(std::move(profile));
    };
  if (!odom_ || !has_grid_ || !reference_.valid()) {
    state_machine_.resetEvidence();
    held_path_.reset();
    PlannerDecisionData unavailable;
    unavailable.requested_intent = state_machine_.state().intent;
    unavailable.proposed_intent = state_machine_.state().intent;
    unavailable.executed_intent = state_machine_.state().intent;
    const bool wants_local_path =
      state_machine_.state().intent != PlannerIntent::FOLLOW_RACING_LINE;
    unavailable.recovery_reason = wants_local_path ?
      RecoveryReason::NO_SAFE_LOCAL_PATH : RecoveryReason::NONE;
    publishDecision(unavailable);
    if (wants_local_path) {publishEmptyLocalPath(cycle_stamp);}
    publishOvertakeReady(wants_local_path);
    finishProfile();
    return;
  }

  const auto odom_started = std::chrono::steady_clock::now();
  const auto odom_in_map = odometryInMap();
  profile.ros.odom_conversion_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - odom_started).count();
  if (!odom_in_map) {
    state_machine_.resetEvidence();
    held_path_.reset();
    PlannerDecisionData unavailable;
    unavailable.requested_intent = state_machine_.state().intent;
    unavailable.proposed_intent = state_machine_.state().intent;
    unavailable.executed_intent = state_machine_.state().intent;
    const bool wants_local_path =
      state_machine_.state().intent != PlannerIntent::FOLLOW_RACING_LINE;
    unavailable.recovery_reason = wants_local_path ?
      RecoveryReason::NO_SAFE_LOCAL_PATH : RecoveryReason::NONE;
    publishDecision(unavailable);
    if (wants_local_path) {publishEmptyLocalPath(cycle_stamp);}
    publishOvertakeReady(wants_local_path);
    finishProfile();
    return;
  }
  profile.outcome.inputs_ready = true;
  Odometry odom = *odom_in_map;
  profile.outcome.steering_fresh = has_steering_ &&
    std::abs((now() - steering_received_).seconds()) <= config_.steering_command_timeout_s;
  if (profile.outcome.steering_fresh) {odom.steering_angle = steering_angle_;}

  const auto state_started = std::chrono::steady_clock::now();
  state_machine_.update(
    odom, grid_, StateUpdateContext{
      cycle_stamp.seconds(), costmap_sequence_, costmap_stamp_s_});
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

  // Ahead of plan() and outside its intent branching on purpose: FOLLOW returns
  // from plan() before generating anything, so this is the only place the debug
  // view can see the maneuver families on a steady lap.
  const auto all_candidates_started = std::chrono::steady_clock::now();
  publishAllCandidates(ego, state_machine_.state(), cycle_stamp);
  profile.ros.marker_publish_ms += std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - all_candidates_started).count();

  const double now_s = cycle_stamp.seconds();
  if (held_path_ && state_machine_.state().intent != held_intent_) {
    held_path_.reset();
  }
  const double hold_age_s = now_s - held_path_stamp_s_;
  CollisionCheckResult held_validation;
  bool held_collision_usable = false;
  const bool held_within_max_age = held_path_ && hold_age_s >= 0.0 &&
    hold_age_s < config_.path_max_hold_s;
  if (held_within_max_age) {
    held_validation = planner_.validatePath(*held_path_, grid_);
    held_collision_usable = held_validation.status == CollisionStatus::FREE ||
      held_validation.status == CollisionStatus::SOFT_INFLATION;
  }
  const bool held_usable = held_within_max_age && held_collision_usable;

  const auto planner_started = std::chrono::steady_clock::now();
  auto result = planner_.plan(state_machine_.state(), ego, grid_, held_usable);
  state_machine_.reportMergeProbe(result.decision.merge_probe_available);
  profile.ros.planner_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - planner_started).count();
  profile.core = result.profile;
  result.decision.start_curvature_from_steering =
    config_.use_steering_start_curvature && profile.outcome.steering_fresh;
  if (result.decision.requested_intent == PlannerIntent::FOLLOW_RACING_LINE) {
    const auto marker_publish_started = std::chrono::steady_clock::now();
    held_path_.reset();
    profile.outcome.decision = result.decision;
    const auto decision_publish_started = std::chrono::steady_clock::now();
    publishDecision(result.decision);
    profile.ros.decision_publish_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - decision_publish_started).count();
    publishOvertakeReady(false);
    profile.ros.marker_publish_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - marker_publish_started).count();
    finishProfile();
    return;
  }
  const Path * publish_path = nullptr;
  const ExecutedMode planned_mode = result.decision.executed_mode;
  const PublishedPathChoice path_choice = choosePublishedPath(
    planned_mode, result.selected_index >= 0, held_path_.has_value(), held_collision_usable,
    hold_age_s, config_.path_min_hold_s, config_.path_max_hold_s);
  if (path_choice == PublishedPathChoice::HELD) {
    publish_path = &*held_path_;
    markHeldPathExecution(result, held_validation, held_executed_intent_);
  } else if (path_choice == PublishedPathChoice::SELECTED) {
    auto & selected = result.pool.at(static_cast<std::size_t>(result.selected_index)).path;
    if (planned_mode == ExecutedMode::MANEUVER) {
      held_path_ = selected;
      held_path_stamp_s_ = now_s;
      held_intent_ = result.decision.requested_intent;
      held_executed_intent_ = result.decision.executed_intent;
      publish_path = &*held_path_;
    } else {
      held_path_.reset();
      publish_path = &selected;
    }
  }
  if (result.decision.requested_intent == PlannerIntent::MERGE) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "MERGE_CHECK requested=1 bounds_ready=%d generated=%u valid=%u "
      "collision_rej=%u out_of_grid_rej=%u track_rej=%u velocity_rej=%u "
      "selected_pool_index=%d executed_mode=%d path_samples=%u collision_poses=%u "
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
  if (result.decision.executed_mode == ExecutedMode::BRAKING_UNAVAILABLE) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kRareErrorThrottleMs,
      "No collision-free braking path; stopping on the last arc");
  }
  profile.outcome.decision = result.decision;
  const auto decision_publish_started = std::chrono::steady_clock::now();
  publishDecision(result.decision);
  profile.ros.decision_publish_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - decision_publish_started).count();
  if (!publish_path) {
    held_path_.reset();
    publishEmptyLocalPath(cycle_stamp);
    publishOvertakeReady(true);
    finishProfile();
    return;
  }

  const auto path_message_started = std::chrono::steady_clock::now();
  const auto map_path = pathToRos(*publish_path, cycle_stamp, config_.map_frame);
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
  visualization_.publishCandidates(result, cycle_stamp, *visualization_pub_);
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

/*
the candidates the current intent generates, drawn before anything filters them.

one family, chosen by intent, mirroring plan()'s generation branch exactly: nothing
is drawn that the planner would not have built this cycle. no recovery families, no
merge probe, no phantom opponent -- those are conditional second passes or previews
of states we are not in, and overlaying them makes the picture a guess.

FOLLOW draws nothing because FOLLOW generates nothing: plan() returns before the
builder is ever called. an empty topic on a steady lap is the honest reading.

what is skipped is only the filtering. every path here is raw builder output, so
one may run through a wall or off the track -- collision, bounds, and velocity all
run inside plan(), on plan()'s own pool, and none of them have touched these.
*/
void PlannerNode::publishAllCandidates(
  const BoundaryState & ego, const TacticalState & state, const rclcpp::Time & stamp)
{
  if (!all_candidates_visualization_pub_ || !reference_.valid()) {
    return;
  }
  std::vector<CandidateFamily> families;
  switch (state.intent) {
    case PlannerIntent::OVERTAKE:
      // Unreachable without a detected opponent, so opponent.s is always a real
      // observation here, never a stand-in.
      families.push_back(
        {CandidateSource::OVERTAKE,
          maneuver_builder_.overtake(ego, state.ego_s, state.ego_d, state.opponent.s)});
      break;
    case PlannerIntent::PASS:
      families.push_back(
        {CandidateSource::PASS_PREFERRED, maneuver_builder_.pass(ego, state.ego_s, state.ego_d)});
      break;
    case PlannerIntent::MERGE:
      families.push_back(
        {CandidateSource::MERGE, maneuver_builder_.merge(ego, state.ego_s, state.ego_d)});
      break;
    case PlannerIntent::FOLLOW_RACING_LINE:
      break;
  }
  visualization_.publishAllCandidates(families, stamp, *all_candidates_visualization_pub_);
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

void PlannerNode::publishEmptyLocalPath(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path empty;
  empty.header.stamp = stamp;
  empty.header.frame_id = config_.map_frame;
  local_path_map_pub_->publish(empty);
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
