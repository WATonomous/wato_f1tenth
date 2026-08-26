// CUDA MPPI ROS 2 node: C++ port of mppi_example/mppi_node.py.
// Opponent handling is fixed to the "clear" mode (radial keep-out cost only);
// the follow/pass auto-overtake state machine was intentionally dropped.
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "mppi_cpp/config.hpp"
#include "mppi_cpp/mppi_cuda.hpp"
#include "mppi_cpp/track.hpp"
#include "mppi_cpp/wall_sdf.hpp"

namespace mppi {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

double wall_now() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
double wrap_angle(double a) { return std::fmod(std::fmod(a + kPi, 2 * kPi) + 2 * kPi, 2 * kPi) - kPi; }
double yaw_of(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
double stamp_to_sec(const builtin_interfaces::msg::Time& t) { return t.sec + t.nanosec * 1e-9; }
bool all_finite(const std::vector<float>& v) {
  return std::all_of(v.begin(), v.end(), [](float x) { return std::isfinite(x); });
}

const std::vector<std::string> kDebugScalars = {
    "reward_total_sum", "reward_total_mean", "reward_xy_sum", "reward_velocity_sum",
    "reward_yaw_sum", "cost_total_sum", "cost_total_mean", "cost_invalid_sum", "cost_wall_sum",
    "cost_slip_sum", "cost_latacc_sum", "cost_steer_sat_sum", "cost_opponent_sum",
    "min_wall_dist", "min_opponent_dist", "opponent_path_age", "opponent_active",
    "callback_wall_dt", "callback_stamp_dt", "prev_pose_dt", "state_est_vy", "state_est_wz",
    "mppi_solve_time", "phase_post_drive_debug", "phase_visualization", "phase_total",
    "mppi_aopt_max_abs", "mppi_saturation_count", "mppi_bad_output_count", "mppi_guard_count",
    "max_beta", "max_latacc", "max_abs_steer", "invalid_steps"};

}  // namespace

class MppiNode : public rclcpp::Node {
 public:
  MppiNode() : Node("mppi_node") {
    declare_params();
    sanitize_params();
    seed_ = cfg_.random_seed < 0 ? std::random_device{}() : static_cast<uint64_t>(cfg_.random_seed);

    std::string wpt = cfg_.wpt_path;
    if (wpt.empty()) throw std::runtime_error("wpt_path is required");
    if (!cfg_.wpt_path_absolute && !std::filesystem::path(wpt).is_absolute())
      wpt = ament_index_cpp::get_package_share_directory("mppi_cpp") + "/data/" + wpt;
    RCLCPP_INFO(get_logger(), "Loading raceline directly from %s", wpt.c_str());
    track_ = std::make_unique<Track>(Track::load_csv(wpt));
    RCLCPP_INFO(get_logger(), "Raceline: %ld waypoints, per-waypoint friction: %s",
                static_cast<long>(track_->waypoints().rows()), track_->has_friction() ? "yes" : "no");

    int model;
    if (cfg_.state_predictor == "dynamic_ST") model = DYNAMIC_ST;
    else if (cfg_.state_predictor == "kinematic_ST") model = KINEMATIC_ST;
    else throw std::runtime_error("unknown state_predictor: " + cfg_.state_predictor);
    RCLCPP_INFO(get_logger(), "MPPI Model: %s", cfg_.state_predictor.c_str());
    mppi_ = std::make_unique<MppiCuda>(cfg_.n_samples, cfg_.n_steps, model,
                                       static_cast<float>(cfg_.sim_time_step), seed_);
    init_wall_cost();
    opp_horizon_.assign(cfg_.n_steps * 2, 0.f);
    est_method_enabled_ = cfg_.use_pose_delta_state_estimate;

    // Dummy solve: initializes the CUDA context so the first real tick is fast.
    {
      const double state0[kStateDim] = {0, 0, 0, 0, 0, 0, 0};
      const float state0f[kStateDim] = {0, 0, 0, 0, 0, 0, 0};
      const RowMat ref = track_->reference(state0, cfg_.ref_vel, cfg_.n_steps, cfg_);
      mppi_->update(state0f, to_float(ref), opp_horizon_,
                    track_->reference_frictions(state0, cfg_.n_steps, cfg_), runtime_params(false));
    }
    RCLCPP_INFO(get_logger(), "MPPI initialized");

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable().durability_volatile();
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(1));
    sensor_qos.best_effort().durability_volatile();
    const rclcpp::QoS debug_qos = sensor_qos;  // BEST_EFFORT so slow viewers can't backpressure

    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        cfg_.is_sim ? "/ego_racecar/odom" : "/pf/pose/odom", sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { pose_callback(std::move(m)); });
    opp_sub_ = create_subscription<nav_msgs::msg::Path>(
        cfg_.opponent_path_topic, sensor_qos,
        [this](nav_msgs::msg::Path::ConstSharedPtr m) { opponent_path_callback(*m); });

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", qos);
    reference_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/reference_arr", debug_qos);
    opt_traj_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/opt_traj_arr", debug_qos);
    speed_debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/mppi/speed_debug", debug_qos);
    reference_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mppi/reference", debug_qos);
    opt_traj_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mppi/optimal_trajectory", debug_qos);
    sampled_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mppi/sampled_trajectories", debug_qos);
    for (const auto& key : kDebugScalars)
      debug_pubs_[key] = create_publisher<std_msgs::msg::Float32>("/mppi/debug/" + key, debug_qos);

    // odom_gate: pose_callback drives control_step, timer is a low-rate watchdog.
    // timer: fixed-rate driver.
    const bool odom_gate = cfg_.control_trigger_mode == "odom_gate";
    const double timer_hz = odom_gate ? cfg_.control_watchdog_hz : cfg_.control_loop_hz;
    const double period = 1.0 / std::max(0.5, timer_hz);
    control_timer_ = create_wall_timer(std::chrono::duration<double>(period), [this] { control_timer(); });
    RCLCPP_INFO(get_logger(),
                "Control trigger mode: %s; timer at %.1f Hz (%s, period %.1f ms); pose-driven target rate %.1f Hz",
                cfg_.control_trigger_mode.c_str(), timer_hz, odom_gate ? "watchdog" : "fixed-rate driver",
                period * 1000.0, cfg_.control_loop_hz);
    if (cfg_.stats_log_interval_sec > 0.0)
      stats_timer_ = create_wall_timer(std::chrono::duration<double>(cfg_.stats_log_interval_sec),
                                       [this] { stats_timer(); });
    params_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { maybe_refresh_params(); });
    stats_window_start_ = wall_now();
  }

 private:
  // ---------------- parameters ----------------
  template <typename T>
  void param(const std::string& name, T& field, const std::string& desc, bool startup = false) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = startup ? "[startup] " + desc : desc;
    d.read_only = startup;
    field = declare_parameter<T>(name, field, d);
    if (!startup)
      live_params_.push_back([this, name, &field] { field = get_parameter(name).get_value<T>(); });
  }

  void declare_params() {
    Config& c = cfg_;
    param("is_sim", c.is_sim, "true: /ego_racecar/odom; false: /pf/pose/odom.", true);
    param("wpt_path_absolute", c.wpt_path_absolute, "Use wpt_path as absolute path to raceline CSV.", true);
    param("wpt_path", c.wpt_path, "Raceline CSV (absolute, or relative to mppi_cpp/data).", true);
    param("state_predictor", c.state_predictor, "Rollout model: dynamic_ST or kinematic_ST.", true);
    param("n_samples", c.n_samples, "Number of sampled control sequences.", true);
    param("n_steps", c.n_steps, "Rollout horizon length.", true);
    param("sim_time_step", c.sim_time_step, "Rollout integration timestep (s).", true);
    param("random_seed", c.random_seed, "Sampling seed (-1 = random each run).", true);
    param("render", c.render, "Roll out a_opt for the optimal-trajectory output (else sample 0).", true);
    param("wall_cost_map_yaml", c.wall_cost_map_yaml, "Map YAML for wall SDF (injected by launch).", true);
    param("opponent_path_topic", c.opponent_path_topic, "Topic publishing opponent predicted Path.", true);
    param("control_loop_hz", c.control_loop_hz, "Control loop / pose-gate rate (Hz).", true);
    param("control_trigger_mode", c.control_trigger_mode, "odom_gate | timer.", true);
    param("control_watchdog_hz", c.control_watchdog_hz, "Watchdog timer rate in odom_gate mode (Hz).", true);

    param("temperature", c.temperature, "MPPI greediness. Lower=winner-take-all; higher=smoother averaging.");
    param("damping", c.damping, "Weight normalization stabilizer when rewards are similar.");
    param("ref_vel", c.ref_vel, "Constant-speed reference seed (bypassed when profile on).");
    param("init_vel", c.init_vel, "Min speed assumed by rollout model at startup/very low speed.");
    param("startup_speed", c.startup_speed, "Min /drive.speed while measured speed < init_vel.");
    param("use_pose_delta_state_estimate", c.use_pose_delta_state_estimate, "Hardware speed estimator from pose deltas.");
    param("friction", c.friction, "Tire/friction belief for dynamic_ST rollouts.");
    param("friction_max", c.friction_max, "Live grip ceiling for per-waypoint / scalar friction.");
    param("n_iterations", c.n_iterations, "MPPI update passes per control step.");

    param("use_waypoint_speed_profile", c.use_waypoint_speed_profile, "Use raceline vx_mps as MPPI reference speed.");
    param("speed_profile_scale", c.speed_profile_scale, "Multiplies raceline vx_mps.");
    param("speed_profile_min_speed", c.speed_profile_min_speed, "Lower clamp for profile speed (m/s).");
    param("speed_profile_max_speed", c.speed_profile_max_speed, "Upper clamp for profile speed (m/s).");
    param("speed_profile_lookahead_steps", c.speed_profile_lookahead_steps, "Planning brake lookahead (steps).");
    param("speed_profile_iterations", c.speed_profile_iterations, "Profile rebuild passes per call.");

    param("use_speed_profile_drive_speed", c.use_speed_profile_drive_speed, "Blend profile speed into /drive.speed.");
    param("speed_profile_drive_blend", c.speed_profile_drive_blend, "0 = pure MPPI accel; 1 = pure profile speed.");
    param("speed_profile_drive_lookahead_steps", c.speed_profile_drive_lookahead_steps, "Command brake lookahead (future ref step).");
    param("speed_profile_drive_use_min_lookahead", c.speed_profile_drive_use_min_lookahead, "Use min speed through lookahead window.");
    param("speed_profile_drive_max_accel", c.speed_profile_drive_max_accel, "Max commanded speed increase rate (m/s^2).");
    param("speed_profile_drive_max_decel", c.speed_profile_drive_max_decel, "Max commanded speed decrease rate (m/s^2).");

    param("control_sample_std_steer", c.control_sample_std_steer, "Bound of normalized steering-rate noise.");
    param("control_sample_std_accel", c.control_sample_std_accel, "Bound of normalized accel noise.");
    param("steer_vel_scale", c.steer_vel_scale, "Converts normalized steering action to rad/s.");
    param("accel_scale", c.accel_scale, "Converts normalized accel action to m/s^2.");

    param("xy_reward_weight", c.xy_reward_weight, "Path-tracking term.");
    param("velocity_reward_weight", c.velocity_reward_weight, "Reference speed mismatch term.");
    param("yaw_reward_weight", c.yaw_reward_weight, "Heading mismatch term.");

    param("wall_cost_enabled", c.wall_cost_enabled, "Enable wall SDF cost.");
    param("wall_cost_weight", c.wall_cost_weight, "Wall cost weight.");
    param("wall_cost_margin", c.wall_cost_margin, "Distance below which wall cost activates (m).");
    param("wall_cost_power", c.wall_cost_power, "Wall cost exponent.");

    param("opponent_cost_enabled", c.opponent_cost_enabled, "Enable opponent keep-out cost.");
    param("opponent_cost_weight", c.opponent_cost_weight, "Opponent proximity cost weight.");
    param("opponent_cost_radius", c.opponent_cost_radius, "Soft keep-out radius around opponent (m).");
    param("opponent_cost_power", c.opponent_cost_power, "Opponent cost exponent.");
    param("opponent_cost_discount", c.opponent_cost_discount, "Per-step discount along opponent horizon.");
    param("opponent_path_timeout", c.opponent_path_timeout, "Stale opponent path timeout (s).");

    param("slip_cost_enabled", c.slip_cost_enabled, "Enable side-slip cost.");
    param("slip_cost_weight", c.slip_cost_weight, "Slip cost weight.");
    param("slip_cost_beta_safe", c.slip_cost_beta_safe, "Safe slip-angle threshold (rad).");
    param("latacc_cost_enabled", c.latacc_cost_enabled, "Enable lateral acceleration cost.");
    param("latacc_cost_weight", c.latacc_cost_weight, "Lat-acc cost weight.");
    param("latacc_cost_safe", c.latacc_cost_safe, "Lat-acc safe threshold (m/s^2).");
    param("steer_sat_cost_enabled", c.steer_sat_cost_enabled, "Enable steering-saturation cost.");
    param("steer_sat_cost_weight", c.steer_sat_cost_weight, "Steering saturation cost weight.");
    param("steer_sat_soft_ratio", c.steer_sat_soft_ratio, "Soft limit as fraction of max steering.");

    param("min_speed", c.min_speed, "Final /drive.speed lower clamp (m/s).");
    param("max_speed", c.max_speed, "Final /drive.speed upper clamp (m/s).");
    param("max_steering_angle", c.max_steering_angle, "Final steering-angle clamp (rad).");

    param("publish_markers", c.publish_markers, "Toggle MPPI MarkerArray publishers.");
    param("marker_frame_id", c.marker_frame_id, "RViz frame for trajectory markers.");
    param("reference_line_width", c.reference_line_width, "Reference (blue) line width (m).");
    param("optimal_line_width", c.optimal_line_width, "Optimal rollout (green) line width (m).");
    param("sampled_line_width", c.sampled_line_width, "Sampled rollout (orange) line width (m).");
    param("sampled_trajectory_count", c.sampled_trajectory_count, "Number of sampled rollouts to draw.");
    param("sampled_trajectory_alpha", c.sampled_trajectory_alpha, "Sampled rollout transparency.");

    param("mppi_guard_on_timing_jump", c.mppi_guard_on_timing_jump, "Guard persistent state on timing jumps.");
    param("mppi_guard_wall_gap", c.mppi_guard_wall_gap, "SOFT wall-time callback gap (s).");
    param("mppi_guard_stamp_gap", c.mppi_guard_stamp_gap, "SOFT odom stamp gap (s).");
    param("mppi_guard_hard_gap", c.mppi_guard_hard_gap, "HARD gap: wipe the warm-start (s).");
    param("mppi_guard_aopt_threshold", c.mppi_guard_aopt_threshold, "Saturation threshold on first actions.");
    param("mppi_guard_saturation_callbacks", c.mppi_guard_saturation_callbacks, "Saturated callbacks before clearing warm-start.");
    param("mppi_guard_bad_callbacks_to_clear_control", c.mppi_guard_bad_callbacks_to_clear_control, "Bad callbacks before startup-speed fallback.");

    param("state_est_vy_prior", c.state_est_vy_prior, "Pose-delta vy IIR prior weight.");
    param("state_est_wz_prior", c.state_est_wz_prior, "Pose-delta wz IIR prior weight.");
    param("state_est_hiccup_dt", c.state_est_hiccup_dt, "Odom dt above this reduces vy/wz prior weights.");
    param("state_est_hiccup_prior_scale", c.state_est_hiccup_prior_scale, "Prior multiplier after an odom timing hiccup.");

    param("control_watchdog_max_silence_sec", c.control_watchdog_max_silence_sec, "Watchdog fires if pose-driven control is silent this long.");
    param("live_tuning_enabled", c.live_tuning_enabled, "Enable 2 Hz live parameter refresh.");
    param("viz_publish_rate_hz", c.viz_publish_rate_hz, "Cap rate for debug/marker publishing (0 disables).");
    param("control_pose_stale_sec", c.control_pose_stale_sec, "Skip MPPI solve if cached pose is older than this (s).");
    param("stats_log_interval_sec", c.stats_log_interval_sec, "Periodic stats logger interval (s). 0 disables.");
  }

  // Clamps from MPPI_Node.get_params.
  void sanitize_params() {
    Config& c = cfg_;
    c.temperature = std::max(1e-6, c.temperature);
    c.damping = std::max(1e-9, c.damping);
    c.n_iterations = std::max(1, c.n_iterations);
    c.steer_vel_scale = std::abs(c.steer_vel_scale);
    c.accel_scale = std::abs(c.accel_scale);
    c.wall_cost_weight = std::max(0.0, c.wall_cost_weight);
    c.wall_cost_margin = std::max(0.0, c.wall_cost_margin);
    c.wall_cost_power = std::max(-5.0, c.wall_cost_power);
    c.opponent_cost_weight = std::max(0.0, c.opponent_cost_weight);
    c.opponent_cost_radius = std::max(0.0, c.opponent_cost_radius);
    c.opponent_cost_power = std::max(0.1, c.opponent_cost_power);
    c.opponent_cost_discount = std::clamp(c.opponent_cost_discount, 0.0, 1.0);
    c.opponent_path_timeout = std::max(0.0, c.opponent_path_timeout);
    c.slip_cost_weight = std::max(0.0, c.slip_cost_weight);
    c.slip_cost_beta_safe = std::max(0.0, c.slip_cost_beta_safe);
    c.latacc_cost_weight = std::max(0.0, c.latacc_cost_weight);
    c.latacc_cost_safe = std::max(0.0, c.latacc_cost_safe);
    c.steer_sat_cost_weight = std::max(0.0, c.steer_sat_cost_weight);
    c.steer_sat_soft_ratio = std::clamp(c.steer_sat_soft_ratio, 0.0, 1.0);
    c.speed_profile_scale = std::max(0.0, c.speed_profile_scale);
    c.speed_profile_min_speed = std::max(0.0, c.speed_profile_min_speed);
    c.speed_profile_max_speed = std::max(c.speed_profile_min_speed, c.speed_profile_max_speed);
    c.speed_profile_lookahead_steps = std::max(0, c.speed_profile_lookahead_steps);
    c.speed_profile_iterations = std::max(1, c.speed_profile_iterations);
    c.speed_profile_drive_blend = std::clamp(c.speed_profile_drive_blend, 0.0, 1.0);
    c.speed_profile_drive_lookahead_steps = std::max(0, c.speed_profile_drive_lookahead_steps);
    c.speed_profile_drive_max_accel = std::max(0.0, c.speed_profile_drive_max_accel);
    c.speed_profile_drive_max_decel = std::max(0.0, c.speed_profile_drive_max_decel);
    c.max_speed = std::max(c.min_speed, c.max_speed);
    c.reference_line_width = std::max(0.001, c.reference_line_width);
    c.optimal_line_width = std::max(0.001, c.optimal_line_width);
    c.sampled_line_width = std::max(0.001, c.sampled_line_width);
    c.sampled_trajectory_count = std::max(0, c.sampled_trajectory_count);
    c.sampled_trajectory_alpha = std::clamp(c.sampled_trajectory_alpha, 0.0, 1.0);
    c.mppi_guard_wall_gap = std::max(0.0, c.mppi_guard_wall_gap);
    c.mppi_guard_stamp_gap = std::max(0.0, c.mppi_guard_stamp_gap);
    c.mppi_guard_hard_gap = std::max(c.mppi_guard_stamp_gap, c.mppi_guard_hard_gap);
    c.mppi_guard_aopt_threshold = std::clamp(c.mppi_guard_aopt_threshold, 0.0, 1.0);
    c.mppi_guard_saturation_callbacks = std::max(1, c.mppi_guard_saturation_callbacks);
    c.mppi_guard_bad_callbacks_to_clear_control = std::max(1, c.mppi_guard_bad_callbacks_to_clear_control);
    c.state_est_vy_prior = std::clamp(c.state_est_vy_prior, 0.0, 0.95);
    c.state_est_wz_prior = std::clamp(c.state_est_wz_prior, 0.0, 0.95);
    c.state_est_hiccup_dt = std::max(0.0, c.state_est_hiccup_dt);
    c.state_est_hiccup_prior_scale = std::clamp(c.state_est_hiccup_prior_scale, 0.0, 1.0);
    c.control_loop_hz = std::clamp(c.control_loop_hz, 1.0, 100.0);
    if (c.control_trigger_mode != "odom_gate" && c.control_trigger_mode != "timer") {
      RCLCPP_WARN(get_logger(), "Unknown control_trigger_mode='%s', falling back to 'odom_gate'",
                  c.control_trigger_mode.c_str());
      c.control_trigger_mode = "odom_gate";
    }
    c.control_watchdog_hz = std::clamp(c.control_watchdog_hz, 0.5, 30.0);
    c.control_watchdog_max_silence_sec = std::max(0.02, c.control_watchdog_max_silence_sec);
    c.control_pose_stale_sec = std::max(0.0, c.control_pose_stale_sec);
    c.stats_log_interval_sec = std::max(0.0, c.stats_log_interval_sec);
    c.viz_publish_rate_hz = std::max(0.0, c.viz_publish_rate_hz);
  }

  void maybe_refresh_params() {
    if (!get_parameter("live_tuning_enabled").as_bool()) return;
    const double t0 = wall_now();
    for (auto& read : live_params_) read();
    sanitize_params();
    init_wall_cost();
    if (est_method_enabled_ != cfg_.use_pose_delta_state_estimate) {
      reset_state_estimator();
      est_method_enabled_ = cfg_.use_pose_delta_state_estimate;
    }
    last_get_params_dt_ = wall_now() - t0;
  }

  void init_wall_cost() {
    const std::string signature = (cfg_.wall_cost_enabled ? "1|" : "0|") + cfg_.wall_cost_map_yaml;
    if (signature == wall_signature_) return;
    wall_signature_ = signature;
    sdf_ = WallSdf{};
    if (cfg_.wall_cost_enabled && !cfg_.wall_cost_map_yaml.empty()) {
      sdf_ = WallSdf::load(cfg_.wall_cost_map_yaml);
      RCLCPP_INFO(get_logger(), "Wall SDF loaded: %dx%d @ %.3f m from %s", sdf_.w, sdf_.h, sdf_.res,
                  cfg_.wall_cost_map_yaml.c_str());
    }
    mppi_->set_wall_sdf(sdf_.data, sdf_.h, sdf_.w, sdf_.ox, sdf_.oy, sdf_.res);
  }

  RuntimeParams runtime_params(bool opponent_active) const {
    const Config& c = cfg_;
    RuntimeParams rp{};
    rp.a_std[0] = c.control_sample_std_steer;
    rp.a_std[1] = c.control_sample_std_accel;
    rp.temperature = c.temperature;
    rp.damping = c.damping;
    rp.reward_weights[0] = c.xy_reward_weight;
    rp.reward_weights[1] = c.velocity_reward_weight;
    rp.reward_weights[2] = c.yaw_reward_weight;
    rp.wall_weight = c.wall_cost_enabled ? c.wall_cost_weight : 0.f;
    rp.wall_margin = c.wall_cost_margin;
    rp.wall_power = c.wall_cost_power;
    rp.slip_weight = c.slip_cost_enabled ? c.slip_cost_weight : 0.f;
    rp.beta_safe = c.slip_cost_beta_safe;
    rp.latacc_weight = c.latacc_cost_enabled ? c.latacc_cost_weight : 0.f;
    rp.latacc_safe = c.latacc_cost_safe;
    rp.steer_sat_weight = c.steer_sat_cost_enabled ? c.steer_sat_cost_weight : 0.f;
    rp.steer_soft = c.steer_sat_soft_ratio * c.max_steering_angle;
    rp.opponent_weight = (c.opponent_cost_enabled && opponent_active) ? c.opponent_cost_weight : 0.f;
    rp.opponent_radius = c.opponent_cost_radius;
    rp.opponent_power = c.opponent_cost_power;
    rp.opponent_discount = c.opponent_cost_discount;
    rp.steer_scale = c.steer_vel_scale;
    rp.accel_scale = c.accel_scale;
    rp.n_iterations = c.n_iterations;
    rp.render = c.render;
    return rp;
  }

  static std::vector<float> to_float(const RowMat& m) {
    std::vector<float> out(m.size());
    for (Eigen::Index i = 0; i < m.rows(); ++i)
      for (Eigen::Index j = 0; j < m.cols(); ++j) out[i * m.cols() + j] = static_cast<float>(m(i, j));
    return out;
  }

  // ---------------- opponent horizon ----------------
  void opponent_path_callback(const nav_msgs::msg::Path& msg) {
    ++stats_opponent_rx_;
    const int T = cfg_.n_steps;
    if (msg.poses.empty()) {
      opp_path_time_.reset();
      opp_horizon_.assign(T * 2, 0.f);
      return;
    }
    const size_t start = msg.poses.size() > 1 ? 1 : 0;
    std::vector<float> pts;
    for (size_t i = start; i < std::min(msg.poses.size(), start + T); ++i) {
      const auto& p = msg.poses[i].pose.position;
      if (std::isfinite(p.x) && std::isfinite(p.y)) { pts.push_back(p.x); pts.push_back(p.y); }
    }
    if (pts.empty()) return;
    while (pts.size() < static_cast<size_t>(T * 2)) {  // pad with the last point
      pts.push_back(pts[pts.size() - 2]);
      pts.push_back(pts[pts.size() - 2]);
    }
    opp_horizon_ = pts;
    opp_path_time_ = now().seconds();
  }

  // Returns (active, age). Horizon is sanitized in place.
  std::pair<bool, double> opponent_status() {
    if (opp_horizon_.size() != static_cast<size_t>(cfg_.n_steps * 2) || !all_finite(opp_horizon_)) {
      opp_horizon_.assign(cfg_.n_steps * 2, 0.f);
      opp_path_time_.reset();
    }
    if (!opp_path_time_) return {false, kInf};
    const double age = now().seconds() - *opp_path_time_;
    return {cfg_.opponent_cost_enabled && age <= cfg_.opponent_path_timeout, age};
  }

  // ---------------- guards ----------------
  void reset_state_estimator() {
    prev_pose_time_.reset();
    est_vx_.reset();
    est_vy_ = 0.0;
    est_wz_ = 0.0;
  }

  void clear_persistent_mppi_state(const std::string& reason, bool reset_estimator = true,
                                   bool clear_control = false) {
    mppi_->reset_warm_start();
    if (reset_estimator) reset_state_estimator();
    if (clear_control) control_ = {0.0, std::max(cfg_.startup_speed, cfg_.min_speed)};
    ++guard_count_;
    RCLCPP_WARN(get_logger(), "Cleared MPPI persistent state (%d): %s", guard_count_, reason.c_str());
  }

  void handle_bad_callback(const std::string& reason) {
    ++bad_output_count_;
    const bool clear_control = bad_output_count_ >= cfg_.mppi_guard_bad_callbacks_to_clear_control;
    clear_persistent_mppi_state(reason, true, clear_control);
    if (clear_control)
      RCLCPP_ERROR(get_logger(), "Falling back to startup command after %d bad callbacks.", bad_output_count_);
  }

  // HARD gaps wipe the warm-start; SOFT gaps only log + count.
  void maybe_guard_for_timing(double wall_dt, double stamp_dt) {
    if (!cfg_.mppi_guard_on_timing_jump) return;
    const double hard = cfg_.mppi_guard_hard_gap;
    char buf[96];
    if (wall_dt > hard) {
      snprintf(buf, sizeof buf, "HARD wall callback gap %.3fs", wall_dt);
      clear_persistent_mppi_state(buf);
      return;
    }
    if (stamp_dt <= 0.0) {
      snprintf(buf, sizeof buf, "non-monotonic odom stamp dt %.3fs", stamp_dt);
      clear_persistent_mppi_state(buf);
      return;
    }
    if (stamp_dt > hard) {
      snprintf(buf, sizeof buf, "HARD odom stamp gap %.3fs", stamp_dt);
      clear_persistent_mppi_state(buf);
      return;
    }
    if (wall_dt > cfg_.mppi_guard_wall_gap || (stamp_dt != 0.0 && stamp_dt > cfg_.mppi_guard_stamp_gap)) {
      ++soft_guard_count_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "SOFT timing gap (warm-start kept): wall=%.3fs stamp=%.3fs", wall_dt, stamp_dt);
    }
  }

  // ---------------- state estimation ----------------
  struct VehicleState { double vx, vy, wz, beta; };

  VehicleState estimate_vehicle_state(const nav_msgs::msg::Odometry& msg, double theta, double callback_time) {
    const auto& tw = msg.twist.twist;
    const double raw_vx = tw.linear.x, raw_vy = tw.linear.y, raw_wz = tw.angular.z;
    const double raw_beta = std::atan2(raw_vy, std::max(std::abs(raw_vx), 1e-6));
    if (cfg_.is_sim || !cfg_.use_pose_delta_state_estimate)
      return {std::max(raw_vx, cfg_.init_vel), raw_vy, raw_wz, raw_beta};

    const double px = msg.pose.pose.position.x, py = msg.pose.pose.position.y;
    const double commanded = std::clamp(control_[1], cfg_.min_speed, cfg_.max_speed);
    auto store_prev = [&] { prev_pose_time_ = callback_time; prev_x_ = px; prev_y_ = py; prev_yaw_ = theta; };

    if (!prev_pose_time_) {
      store_prev();
      const double seed = std::max({raw_vx, commanded, cfg_.init_vel});
      est_vx_ = seed;
      est_vy_ = raw_vy;
      est_wz_ = raw_wz;
      timing_["prev_pose_dt"] = 0.0;
      timing_["state_est_vy"] = raw_vy;
      timing_["state_est_wz"] = raw_wz;
      return {seed, raw_vy, raw_wz, std::atan2(raw_vy, std::max(seed, 1e-6))};
    }

    const double dt = callback_time - *prev_pose_time_;
    if (!std::isfinite(dt) || dt <= 1e-3 || dt > 0.5) {
      store_prev();
      timing_["prev_pose_dt"] = std::isfinite(dt) ? dt : -1.0;
      const double vx = std::max({raw_vx, commanded, est_vx_.value_or(cfg_.init_vel)});
      return {vx, est_vy_, est_wz_, std::atan2(est_vy_, std::max(std::abs(vx), 1e-6))};
    }
    timing_["prev_pose_dt"] = dt;

    const double wx = (px - prev_x_) / dt, wy = (py - prev_y_) / dt;
    const double c = std::cos(theta), s = std::sin(theta);
    const double vx_pose = std::clamp(c * wx + s * wy, -cfg_.max_speed, cfg_.max_speed);
    const double vy_pose = std::clamp(-s * wx + c * wy, -3.0, 3.0);
    const double wz_pose = std::clamp(wrap_angle(theta - prev_yaw_) / dt, -8.0, 8.0);

    const double speed_obs = std::isfinite(raw_vx) ? 0.5 * (vx_pose + raw_vx) : vx_pose;
    const double prev_vx = est_vx_.value_or(speed_obs);
    double vx_est = 0.55 * speed_obs + 0.30 * commanded + 0.15 * prev_vx;
    vx_est = std::max(std::clamp(vx_est, cfg_.min_speed, cfg_.max_speed), cfg_.init_vel);

    double prior_scale = 1.0;
    if (cfg_.state_est_hiccup_dt > 0.0 && dt > cfg_.state_est_hiccup_dt) prior_scale = cfg_.state_est_hiccup_prior_scale;
    const double vy_prior = std::clamp(cfg_.state_est_vy_prior * prior_scale, 0.0, 0.95);
    const double wz_prior = std::clamp(cfg_.state_est_wz_prior * prior_scale, 0.0, 0.95);

    const double vy_obs = 0.8 * vy_pose + 0.2 * raw_vy;
    const double vy_est = std::clamp((1.0 - vy_prior) * vy_obs + vy_prior * est_vy_, -2.0, 2.0);
    const double wz_obs = 0.85 * wz_pose + 0.15 * raw_wz;
    const double wz_est = std::clamp((1.0 - wz_prior) * wz_obs + wz_prior * est_wz_, -6.0, 6.0);

    store_prev();
    est_vx_ = vx_est;
    est_vy_ = vy_est;
    est_wz_ = wz_est;
    timing_["state_est_vy"] = vy_est;
    timing_["state_est_wz"] = wz_est;
    return {vx_est, vy_est, wz_est, std::atan2(vy_est, std::max(std::abs(vx_est), 1e-6))};
  }

  // ---------------- triggers ----------------
  void pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    latest_pose_ = msg;
    const double now_w = wall_now();
    latest_pose_recv_time_ = now_w;
    ++stats_pose_rx_;
    if (cfg_.control_trigger_mode != "odom_gate") return;
    if (now_w < next_control_fire_time_) return;
    stats_pose_age_ = 0.0;
    const double period = 1.0 / std::max(1.0, cfg_.control_loop_hz);
    next_control_fire_time_ = std::max(now_w + period * 0.5, next_control_fire_time_ + period);
    last_control_step_start_ = now_w;
    ++stats_trigger_odom_;
    control_step(*msg);
  }

  void control_timer() {
    ++stats_control_ticks_;
    auto msg = latest_pose_;
    if (!msg) { ++stats_skips_no_pose_; return; }
    const double now_w = wall_now();
    if (cfg_.control_trigger_mode == "odom_gate") {
      if (now_w - last_control_step_start_ < cfg_.control_watchdog_max_silence_sec) return;
      ++stats_trigger_watchdog_;
    }
    const double recv_age = now_w - latest_pose_recv_time_;
    stats_pose_age_ = recv_age;
    if (recv_age > cfg_.control_pose_stale_sec) { ++stats_skips_stale_; return; }
    last_control_step_start_ = now_w;
    control_step(*msg);
  }

  // ---------------- the control step ----------------
  void control_step(const nav_msgs::msg::Odometry& msg) {
    const double t1 = wall_now();
    const auto& pose = msg.pose.pose;
    const auto& twist = msg.twist.twist;
    const double theta = yaw_of(pose.orientation);
    double callback_time = stamp_to_sec(msg.header.stamp);
    if (callback_time <= 0.0) callback_time = t1;
    if (!std::isfinite(callback_time)) return;
    for (double v : {pose.position.x, pose.position.y, theta, twist.linear.x, twist.linear.y, twist.angular.z})
      if (!std::isfinite(v)) { handle_bad_callback("non-finite odom input"); return; }

    const double wall_dt = last_callback_wall_time_ ? t1 - *last_callback_wall_time_ : 0.0;
    const double stamp_dt = last_pose_msg_time_ ? callback_time - *last_pose_msg_time_ : 0.0;
    last_callback_wall_time_ = t1;
    last_pose_msg_time_ = callback_time;
    timing_["callback_wall_dt"] = wall_dt;
    timing_["callback_stamp_dt"] = stamp_dt;
    if (wall_dt > 0.0 && stamp_dt != 0.0) maybe_guard_for_timing(wall_dt, stamp_dt);

    const VehicleState vs = estimate_vehicle_state(msg, theta, callback_time);
    const double state[kStateDim] = {pose.position.x, pose.position.y, control_[0], vs.vx, theta, vs.wz, vs.beta};
    for (double v : state)
      if (!std::isfinite(v)) { handle_bad_callback("non-finite estimated state"); return; }

    const double find_waypoint_vel = std::max(cfg_.ref_vel, vs.vx);
    const RowMat reference = track_->reference(state, find_waypoint_vel, cfg_.n_steps, cfg_);
    if (!reference.allFinite()) { handle_bad_callback("non-finite reference trajectory"); return; }
    const auto [opponent_active, opponent_age] = opponent_status();

    // ---- MPPI solve ----
    const double solve_t0 = wall_now();
    const std::vector<float> mu = track_->reference_frictions(state, cfg_.n_steps, cfg_);
    float statef[kStateDim];
    for (int i = 0; i < kStateDim; ++i) statef[i] = static_cast<float>(state[i]);
    const std::vector<float> ref_f = to_float(reference);
    mppi_->update(statef, ref_f, opp_horizon_, mu, runtime_params(opponent_active));
    std::vector<float> a_opt = mppi_->a_opt();
    timing_["mppi_solve_time"] = wall_now() - solve_t0;

    double aopt_max_abs = 0.0;
    for (float a : a_opt) if (std::isfinite(a)) aopt_max_abs = std::max(aopt_max_abs, std::abs(static_cast<double>(a)));
    if (!all_finite(a_opt)) aopt_max_abs = kInf;
    timing_["mppi_aopt_max_abs"] = aopt_max_abs;

    bool bad_mppi_output = false;
    if (!all_finite(a_opt) || !all_finite(mppi_->traj_opt())) {
      bad_mppi_output = true;
      handle_bad_callback("non-finite MPPI output");
      a_opt = mppi_->a_opt();
    } else if (aopt_max_abs >= cfg_.mppi_guard_aopt_threshold) {
      const int rows = std::min(3, cfg_.n_steps);
      int sat = 0;
      for (int i = 0; i < rows * kActionDim; ++i) sat += std::abs(a_opt[i]) >= cfg_.mppi_guard_aopt_threshold;
      const double saturated_frac = static_cast<double>(sat) / (rows * kActionDim);
      if (saturated_frac > 0.8) {
        if (++saturation_count_ >= cfg_.mppi_guard_saturation_callbacks) {
          char buf[96];
          snprintf(buf, sizeof buf, "warm start saturation max=%.3f frac=%.2f", aopt_max_abs, saturated_frac);
          clear_persistent_mppi_state(buf, false);
          saturation_count_ = 0;
          a_opt = mppi_->a_opt();
        }
      } else {
        saturation_count_ = 0;
      }
    } else {
      saturation_count_ = 0;
    }
    if (!bad_mppi_output && all_finite(a_opt)) bad_output_count_ = 0;

    // Visualization-only rollout from raw twist so the marker is not perturbed
    // by IIR estimator noise; the controller still used the estimated state.
    viz_traj_opt_.reset();
    if (cfg_.render && cfg_.use_pose_delta_state_estimate && !bad_mppi_output) {
      float viz_state[kStateDim];
      for (int i = 0; i < kStateDim; ++i) viz_state[i] = statef[i];
      const double raw_vx = std::max(twist.linear.x, cfg_.init_vel);
      viz_state[3] = raw_vx;
      viz_state[5] = twist.angular.z;
      viz_state[6] = std::atan2(twist.linear.y, std::max(std::abs(raw_vx), 1e-6));
      viz_traj_opt_ = mppi_->rollout_host(a_opt, viz_state, std::vector<float>(cfg_.n_steps, cfg_.friction),
                                          cfg_.steer_vel_scale, cfg_.accel_scale);
    }

    // ---- compose /drive ----
    const double steer_rate = a_opt[0] * cfg_.steer_vel_scale;
    const double accel = a_opt[1] * cfg_.accel_scale;
    const double prev_speed_command = control_[1];
    double mppi_speed_command = accel * cfg_.sim_time_step + vs.vx;
    double profile_speed_command = std::nan("");
    double speed_command = mppi_speed_command;

    if (cfg_.use_waypoint_speed_profile && cfg_.use_speed_profile_drive_speed) {
      const int idx = std::min<int>(cfg_.speed_profile_drive_lookahead_steps, reference.rows() - 1);
      if (cfg_.speed_profile_drive_use_min_lookahead) {
        profile_speed_command = reference.col(2).head(idx + 1).minCoeff();
      } else {
        profile_speed_command = reference(idx, 2);
      }
      const double blend = cfg_.speed_profile_drive_blend;
      if (std::isnan(mppi_speed_command)) mppi_speed_command = prev_speed_command;
      speed_command = blend >= 1.0 - 1e-6 ? profile_speed_command
                                          : (1.0 - blend) * mppi_speed_command + blend * profile_speed_command;
      const double dt = last_speed_command_time_ ? std::max(0.0, t1 - *last_speed_command_time_) : cfg_.sim_time_step;
      const double max_accel_step = cfg_.speed_profile_drive_max_accel * dt;
      const double max_decel_step = cfg_.speed_profile_drive_max_decel * dt;
      if (max_accel_step > 0.0 && speed_command > prev_speed_command)
        speed_command = std::min(speed_command, prev_speed_command + max_accel_step);
      if (max_decel_step > 0.0 && speed_command < prev_speed_command)
        speed_command = std::max(speed_command, prev_speed_command - max_decel_step);
    }
    last_speed_command_time_ = t1;

    control_[0] = std::clamp(steer_rate * cfg_.sim_time_step + control_[0], -cfg_.max_steering_angle, cfg_.max_steering_angle);
    control_[1] = std::clamp(speed_command, cfg_.min_speed, cfg_.max_speed);
    if (vs.vx < cfg_.init_vel)
      control_[1] = std::max(control_[1], std::clamp(cfg_.startup_speed, cfg_.min_speed, cfg_.max_speed));
    if (!std::isfinite(control_[0]) || !std::isfinite(control_[1])) {
      control_ = {0.0, 0.0};
      mppi_->reset_warm_start();
    }

    // Safety-critical publish first; debug/viz work never sits in front of it.
    ackermann_msgs::msg::AckermannDriveStamped drive;
    drive.header.stamp = now();
    drive.header.frame_id = "base_link";
    drive.drive.steering_angle = control_[0];
    drive.drive.speed = control_[1];
    drive_pub_->publish(drive);
    ++stats_drive_tx_;
    stats_solve_times_.push_back(wall_now() - t1);
    if (stats_solve_times_.size() > 1024)
      stats_solve_times_.erase(stats_solve_times_.begin(), stats_solve_times_.end() - 256);

    // ---- post-drive debug + visualization (rate-gated, subscriber-gated) ----
    double debug_dt = 0.0, viz_dt = 0.0;
    if (cfg_.viz_publish_rate_hz > 0.0) {
      const double now_t = wall_now();
      if (now_t - last_viz_pub_time_ >= 1.0 / cfg_.viz_publish_rate_hz) {
        last_viz_pub_time_ = now_t;
        if (reference_pub_->get_subscription_count() > 0)
          reference_pub_->publish(to_multiarray(ref_f, reference.rows(), kRefDim));
        if (opt_traj_pub_->get_subscription_count() > 0)
          opt_traj_pub_->publish(to_multiarray(mppi_->traj_opt(), cfg_.n_steps, kStateDim));

        const double debug_t0 = wall_now();
        publish_reward_debug(reference, opponent_active, opponent_age);
        if (speed_debug_pub_->get_subscription_count() > 0) {
          const std::vector<float> sd = {static_cast<float>(vs.vx), static_cast<float>(mppi_speed_command),
                                         static_cast<float>(profile_speed_command), static_cast<float>(control_[1]),
                                         static_cast<float>(cfg_.speed_profile_drive_blend)};
          speed_debug_pub_->publish(to_multiarray(sd, 5, 0));
        }
        debug_dt = wall_now() - debug_t0;
        const double viz_t0 = wall_now();
        publish_visualization(reference);
        viz_dt = wall_now() - viz_t0;
      }
    }
    const double total_dt = wall_now() - t1;
    timing_["phase_post_drive_debug"] = debug_dt;
    timing_["phase_visualization"] = viz_dt;
    timing_["phase_total"] = total_dt;
    stats_phase_total_max_ = std::max(stats_phase_total_max_, total_dt);
    stats_phase_debug_max_ = std::max(stats_phase_debug_max_, debug_dt);
    stats_phase_viz_max_ = std::max(stats_phase_viz_max_, viz_dt);
  }

  // ---------------- debug outputs ----------------
  static std_msgs::msg::Float32MultiArray to_multiarray(const std::vector<float>& data, int rows, int cols) {
    std_msgs::msg::Float32MultiArray msg;
    int i = 0;
    for (int size : {rows, cols}) {
      if (size <= 0) break;
      std_msgs::msg::MultiArrayDimension d;
      d.label = "dim" + std::to_string(i++);
      d.size = size;
      d.stride = size * sizeof(float);
      msg.layout.dim.push_back(d);
    }
    msg.data = data;
    return msg;
  }

  // InferEnv.reward_debug_terms evaluated on the optimal trajectory (host).
  void publish_reward_debug(const RowMat& reference, bool opponent_active, double opponent_age) {
    bool any = false;
    for (const auto& [k, pub] : debug_pubs_) any = any || pub->get_subscription_count() > 0;
    if (!any) return;

    const RuntimeParams rp = runtime_params(opponent_active);
    const std::vector<float>& traj = mppi_->traj_opt();
    const int n = std::min<int>(cfg_.n_steps, reference.rows() - 1);
    std::map<std::string, double> d;
    double r_xy = 0, r_vel = 0, r_yaw = 0, c_inv = 0, c_wall = 0, c_slip = 0, c_lat = 0, c_steer = 0, c_opp = 0;
    double min_wall = kInf, min_opp = kInf, max_beta = 0, max_lat = 0, max_steer = 0;
    int invalid_steps = 0;
    for (int t = 0; t < n; ++t) {
      double s[kStateDim];
      bool finite = true;
      for (int i = 0; i < kStateDim; ++i) {
        const float v = traj[t * kStateDim + i];
        finite = finite && std::isfinite(v);
        s[i] = std::isnan(v) ? 1e3 : std::isinf(v) ? (v > 0 ? 1e3 : -1e3) : v;
      }
      const double inv = finite ? 0.0 : 1e3;
      invalid_steps += !finite;
      const double rx = reference(t + 1, 0), ry = reference(t + 1, 1), rv = reference(t + 1, 2), ryaw = reference(t + 1, 3);
      r_xy += rp.reward_weights[0] * -(std::abs(rx - s[0]) + std::abs(ry - s[1]));
      r_vel += rp.reward_weights[1] * -std::abs(rv - s[3]);
      r_yaw += rp.reward_weights[2] * (-std::abs(std::sin(ryaw) - std::sin(s[4])) - std::abs(std::cos(ryaw) - std::cos(s[4])));
      c_inv += inv;
      const double wall = sdf_.sample(s[0], s[1]);
      min_wall = std::min(min_wall, wall);
      c_wall += rp.wall_weight * std::pow(std::max(0.0, rp.wall_margin - wall), rp.wall_power);
      auto hinge = [](double v, double thr) { const double x = std::min(std::max(0.0, v - thr), 1e3); return x * x; };
      const double beta = std::abs(s[6]), lat = std::abs(s[3] * s[5]), steer = std::abs(s[2]);
      max_beta = std::max(max_beta, beta);
      max_lat = std::max(max_lat, lat);
      max_steer = std::max(max_steer, steer);
      c_slip += rp.slip_weight * hinge(beta, rp.beta_safe);
      c_lat += rp.latacc_weight * hinge(lat, rp.latacc_safe);
      c_steer += rp.steer_sat_weight * hinge(steer, rp.steer_soft);
      const double od = std::hypot(s[0] - opp_horizon_[2 * t], s[1] - opp_horizon_[2 * t + 1]);
      min_opp = std::min(min_opp, od);
      c_opp += rp.opponent_weight * std::pow(rp.opponent_discount, t) * std::pow(std::max(0.0, rp.opponent_radius - od), rp.opponent_power);
    }
    const double cost_total = c_inv + c_wall + c_slip + c_lat + c_steer + c_opp;
    const double reward_total = r_xy + r_vel + r_yaw - cost_total;
    const double denom = std::max(1, n);
    d["reward_total_sum"] = reward_total;
    d["reward_total_mean"] = reward_total / denom;
    d["reward_xy_sum"] = r_xy;
    d["reward_velocity_sum"] = r_vel;
    d["reward_yaw_sum"] = r_yaw;
    d["cost_total_sum"] = cost_total;
    d["cost_total_mean"] = cost_total / denom;
    d["cost_invalid_sum"] = c_inv;
    d["cost_wall_sum"] = c_wall;
    d["cost_slip_sum"] = c_slip;
    d["cost_latacc_sum"] = c_lat;
    d["cost_steer_sat_sum"] = c_steer;
    d["cost_opponent_sum"] = c_opp;
    d["min_wall_dist"] = n > 0 ? min_wall : 100.0;
    d["min_opponent_dist"] = opponent_active ? (n > 0 ? min_opp : 100.0) : -1.0;
    d["max_beta"] = max_beta;
    d["max_latacc"] = max_lat;
    d["max_abs_steer"] = max_steer;
    d["invalid_steps"] = invalid_steps;
    d["opponent_path_age"] = std::isfinite(opponent_age) ? opponent_age : -1.0;
    d["opponent_active"] = opponent_active ? 1.0 : 0.0;
    for (const auto& [k, v] : timing_) d[k] = v;
    d["mppi_saturation_count"] = saturation_count_;
    d["mppi_bad_output_count"] = bad_output_count_;
    d["mppi_guard_count"] = guard_count_;

    for (const auto& [key, pub] : debug_pubs_) {
      if (pub->get_subscription_count() == 0) continue;
      std_msgs::msg::Float32 m;
      const auto it = d.find(key);
      m.data = it == d.end() ? 0.f : static_cast<float>(it->second);
      pub->publish(m);
    }
  }

  visualization_msgs::msg::Marker make_line_strip(const std::string& ns, int id, const float* xy, int n,
                                                  int stride, std::array<float, 4> rgba, double width,
                                                  const builtin_interfaces::msg::Time& stamp, double z) const {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = cfg_.marker_frame_id;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = width;
    m.color.r = rgba[0]; m.color.g = rgba[1]; m.color.b = rgba[2]; m.color.a = rgba[3];
    for (int i = 0; i < n; ++i) {
      const float x = xy[i * stride], y = xy[i * stride + 1];
      if (!std::isfinite(x) || !std::isfinite(y)) continue;
      geometry_msgs::msg::Point p;
      p.x = x; p.y = y; p.z = z;
      m.points.push_back(p);
    }
    return m;
  }

  void publish_visualization(const RowMat& reference) {
    if (!cfg_.publish_markers) return;
    const auto stamp = now();
    if (reference_marker_pub_->get_subscription_count() > 0) {
      const std::vector<float> ref_f = to_float(reference);
      visualization_msgs::msg::MarkerArray arr;
      arr.markers.push_back(make_line_strip("mppi_reference", 0, ref_f.data(), reference.rows(), kRefDim,
                                            {0.05f, 0.35f, 1.0f, 1.0f}, cfg_.reference_line_width, stamp, 0.06));
      reference_marker_pub_->publish(arr);
    }
    if (opt_traj_marker_pub_->get_subscription_count() > 0) {
      const std::vector<float>& traj = viz_traj_opt_ ? *viz_traj_opt_ : mppi_->traj_opt();
      visualization_msgs::msg::MarkerArray arr;
      arr.markers.push_back(make_line_strip("mppi_optimal_trajectory", 0, traj.data(), cfg_.n_steps, kStateDim,
                                            {0.0f, 0.95f, 0.2f, 1.0f}, cfg_.optimal_line_width, stamp, 0.08));
      opt_traj_marker_pub_->publish(arr);
    }
    if (sampled_marker_pub_->get_subscription_count() > 0) {
      visualization_msgs::msg::MarkerArray arr;
      visualization_msgs::msg::Marker del;
      del.header.frame_id = cfg_.marker_frame_id;
      del.header.stamp = stamp;
      del.ns = "mppi_sampled_trajectories";
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(del);
      const int max_count = cfg_.sampled_trajectory_count;
      if (max_count > 0) {
        const std::vector<float> states = mppi_->sampled_states();
        const int stride_k = std::max(1, static_cast<int>(std::ceil(static_cast<double>(cfg_.n_samples) / max_count)));
        int id = 0;
        for (int k = 0; k < cfg_.n_samples && id < max_count; k += stride_k, ++id)
          arr.markers.push_back(make_line_strip("mppi_sampled_trajectories", id,
                                                states.data() + static_cast<size_t>(k) * cfg_.n_steps * kStateDim,
                                                cfg_.n_steps, kStateDim,
                                                {1.0f, 0.42f, 0.05f, static_cast<float>(cfg_.sampled_trajectory_alpha)},
                                                cfg_.sampled_line_width, stamp, 0.04));
      }
      sampled_marker_pub_->publish(arr);
    }
  }

  void stats_timer() {
    if (cfg_.stats_log_interval_sec <= 0.0) return;
    const double now_w = wall_now();
    const double window = now_w - stats_window_start_;
    if (window <= 0.0) return;
    double mean = 0, p99 = 0, mx = 0;
    if (!stats_solve_times_.empty()) {
      std::vector<double> s = stats_solve_times_;
      std::sort(s.begin(), s.end());
      for (double v : s) mean += v;
      mean /= s.size();
      p99 = s[std::min(s.size() - 1, static_cast<size_t>(std::llround(0.99 * (s.size() - 1))))];
      mx = s.back();
    }
    const int trigger_total = std::max(1, stats_trigger_odom_ + stats_trigger_watchdog_);
    RCLCPP_INFO(get_logger(),
                "MPPI %5.1fHz | pose_rx %5.1fHz | drive_tx %5.1fHz | ctrl_ticks %5.1fHz (skip stale=%d no_pose=%d) | "
                "trigger odom=%d wdog=%d (%.0f%% odom) | solve mean=%5.1fms p99=%5.1fms max=%5.1fms | "
                "phase_total_max=%5.1fms (debug=%.1f viz=%.1f) | pose_age=%5.1fms | guard+=%d (soft+=%d) | "
                "get_params_dt=%.1fms | opp_rx %4.1fHz",
                stats_drive_tx_ / window, stats_pose_rx_ / window, stats_drive_tx_ / window,
                stats_control_ticks_ / window, stats_skips_stale_, stats_skips_no_pose_, stats_trigger_odom_,
                stats_trigger_watchdog_, 100.0 * stats_trigger_odom_ / trigger_total, mean * 1e3, p99 * 1e3,
                mx * 1e3, stats_phase_total_max_ * 1e3, stats_phase_debug_max_ * 1e3, stats_phase_viz_max_ * 1e3,
                stats_pose_age_ * 1e3, guard_count_ - stats_guard_at_window_start_,
                soft_guard_count_ - stats_soft_guard_at_window_start_, last_get_params_dt_ * 1e3,
                stats_opponent_rx_ / window);
    stats_window_start_ = now_w;
    stats_pose_rx_ = stats_drive_tx_ = stats_control_ticks_ = stats_skips_stale_ = stats_skips_no_pose_ = 0;
    stats_trigger_odom_ = stats_trigger_watchdog_ = stats_opponent_rx_ = 0;
    stats_solve_times_.clear();
    stats_phase_total_max_ = stats_phase_debug_max_ = stats_phase_viz_max_ = 0.0;
    stats_guard_at_window_start_ = guard_count_;
    stats_soft_guard_at_window_start_ = soft_guard_count_;
  }

  // ---------------- members ----------------
  Config cfg_;
  std::vector<std::function<void()>> live_params_;
  uint64_t seed_ = 0;
  std::unique_ptr<Track> track_;
  std::unique_ptr<MppiCuda> mppi_;
  WallSdf sdf_;
  std::string wall_signature_;

  std::array<double, 2> control_{0.0, 0.0};  // steering angle, speed
  std::vector<float> opp_horizon_;
  std::optional<double> opp_path_time_;
  std::optional<std::vector<float>> viz_traj_opt_;

  nav_msgs::msg::Odometry::ConstSharedPtr latest_pose_;
  double latest_pose_recv_time_ = 0.0, last_control_step_start_ = 0.0, next_control_fire_time_ = 0.0;
  std::optional<double> last_callback_wall_time_, last_pose_msg_time_, last_speed_command_time_;
  double last_viz_pub_time_ = 0.0, last_get_params_dt_ = 0.0;

  int guard_count_ = 0, soft_guard_count_ = 0, saturation_count_ = 0, bad_output_count_ = 0;
  std::optional<double> prev_pose_time_, est_vx_;
  double prev_x_ = 0, prev_y_ = 0, prev_yaw_ = 0, est_vy_ = 0.0, est_wz_ = 0.0;
  bool est_method_enabled_ = false;
  std::map<std::string, double> timing_;

  double stats_window_start_ = 0.0, stats_pose_age_ = 0.0;
  int stats_pose_rx_ = 0, stats_drive_tx_ = 0, stats_control_ticks_ = 0, stats_skips_stale_ = 0;
  int stats_skips_no_pose_ = 0, stats_trigger_odom_ = 0, stats_trigger_watchdog_ = 0, stats_opponent_rx_ = 0;
  int stats_guard_at_window_start_ = 0, stats_soft_guard_at_window_start_ = 0;
  std::vector<double> stats_solve_times_;
  double stats_phase_total_max_ = 0.0, stats_phase_debug_max_ = 0.0, stats_phase_viz_max_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr opp_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr reference_pub_, opt_traj_pub_, speed_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr reference_marker_pub_, opt_traj_marker_pub_, sampled_marker_pub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> debug_pubs_;
  rclcpp::TimerBase::SharedPtr control_timer_, stats_timer_, params_timer_;
};

}  // namespace mppi

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mppi::MppiNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
