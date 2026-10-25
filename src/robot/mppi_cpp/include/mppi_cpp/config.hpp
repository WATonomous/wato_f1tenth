#pragma once
#include <string>

namespace mppi {

// Defaults mirror mppi_example/config.yaml + MPPI_Node.ensure_config_defaults.
// Every field is overridable through ROS params (see MppiNode::declare_params).
struct Config {
  // ---- startup (read once) ----
  bool is_sim = true;
  bool wpt_path_absolute = false;
  std::string wpt_path;
  std::string state_predictor = "dynamic_ST";  // dynamic_ST | kinematic_ST
  int n_samples = 1024;
  int n_steps = 10;
  double sim_time_step = 0.1;
  int random_seed = 527787;  // -1 = random
  bool render = true;
  std::string wall_cost_map_yaml;
  std::string opponent_path_topic = "/opponent/predicted_path";
  double control_loop_hz = 25.0;
  std::string control_trigger_mode = "odom_gate";  // odom_gate | timer
  double control_watchdog_hz = 5.0;

  // ---- live ----
  double temperature = 0.01;
  double damping = 0.001;
  double ref_vel = 2.0;
  double init_vel = 1.0;
  double startup_speed = 2.0;
  bool use_pose_delta_state_estimate = false;
  double friction = 0.8;
  double friction_max = 1.5;
  int n_iterations = 1;

  bool use_waypoint_speed_profile = false;
  double speed_profile_scale = 1.0;
  double speed_profile_min_speed = 0.0;
  double speed_profile_max_speed = 20.0;
  int speed_profile_lookahead_steps = 0;
  int speed_profile_iterations = 1;

  bool use_speed_profile_drive_speed = false;
  double speed_profile_drive_blend = 0.0;
  int speed_profile_drive_lookahead_steps = 1;
  bool speed_profile_drive_use_min_lookahead = false;
  double speed_profile_drive_max_accel = 0.0;
  double speed_profile_drive_max_decel = 0.0;

  double control_sample_std_steer = 0.5;
  double control_sample_std_accel = 0.5;
  double steer_vel_scale = 3.0;  // normalized steer-rate +1 -> rad/s
  double accel_scale = 4.0;      // normalized accel +1 -> m/s^2

  double xy_reward_weight = 1.0;
  double velocity_reward_weight = 0.0;
  double yaw_reward_weight = 0.0;

  bool wall_cost_enabled = false;
  double wall_cost_weight = 0.0;
  double wall_cost_margin = 0.3;
  double wall_cost_power = 0.01;

  bool opponent_cost_enabled = false;
  double opponent_cost_weight = 0.0;
  double opponent_cost_radius = 0.8;
  double opponent_cost_power = 2.0;
  double opponent_cost_discount = 1.0;
  double opponent_path_timeout = 0.5;

  bool slip_cost_enabled = false;
  double slip_cost_weight = 0.0;
  double slip_cost_beta_safe = 0.2;
  bool latacc_cost_enabled = false;
  double latacc_cost_weight = 0.0;
  double latacc_cost_safe = 8.0;
  bool steer_sat_cost_enabled = false;
  double steer_sat_cost_weight = 0.0;
  double steer_sat_soft_ratio = 0.85;

  double min_speed = 0.0;
  double max_speed = 20.0;
  double max_steering_angle = 0.4189;

  bool publish_markers = false;
  std::string marker_frame_id = "map";
  double reference_line_width = 0.06;
  double optimal_line_width = 0.08;
  double sampled_line_width = 0.025;
  int sampled_trajectory_count = 0;
  double sampled_trajectory_alpha = 0.18;

  bool mppi_guard_on_timing_jump = true;
  double mppi_guard_wall_gap = 0.25;
  double mppi_guard_stamp_gap = 0.25;
  double mppi_guard_hard_gap = 1.5;
  double mppi_guard_aopt_threshold = 0.98;
  int mppi_guard_saturation_callbacks = 4;
  int mppi_guard_bad_callbacks_to_clear_control = 3;

  double state_est_vy_prior = 0.40;
  double state_est_wz_prior = 0.40;
  double state_est_hiccup_dt = 0.06;
  double state_est_hiccup_prior_scale = 0.35;

  double control_watchdog_max_silence_sec = 0.10;
  double control_pose_stale_sec = 0.20;
  double stats_log_interval_sec = 5.0;
  bool live_tuning_enabled = false;
  double viz_publish_rate_hz = 5.0;
};

}  // namespace mppi
