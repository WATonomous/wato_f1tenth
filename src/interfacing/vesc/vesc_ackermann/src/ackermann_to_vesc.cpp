// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using f1tenth_msgs::msg::SpeedLimiterStatus;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options)
{
  // declare conversion parameters
  declare_parameter("speed_to_erpm_gain", 0.0);
  declare_parameter("speed_to_erpm_offset", 0.0);
  declare_parameter("steering_angle_to_servo_gain", 0.0);
  declare_parameter("steering_angle_to_servo_offset", 0.0);

  // speed limiter + feedforward traction control + stall guard (off unless set in YAML)
  declare_parameter("max_accel", 0.0);
  declare_parameter("max_decel", 0.0);
  declare_parameter("launch_accel", 0.0);
  declare_parameter("launch_blend_speed", 0.0);
  declare_parameter("max_speed_lead", 0.0);
  declare_parameter("grip_accel_limit", 0.0);
  declare_parameter("min_corner_accel", 0.0);
  declare_parameter("wheelbase", 0.0);
  declare_parameter("high_steer_angle", 0.0);
  declare_parameter("high_steer_launch_accel", 0.0);
  declare_parameter("stall_cmd_min_speed", 0.3);
  declare_parameter("stall_measured_max_speed", 0.05);
  declare_parameter("stall_min_motor_current", 20.0);
  declare_parameter("stall_detect_time", 0.0);
  declare_parameter("stall_reset_duration", 0.15);
  declare_parameter("stall_max_retries", 3);
  declare_parameter("cmd_timeout", 0.5);
  declare_parameter("vesc_state_timeout", 0.25);
  declare_parameter("limiter_status_topic", "/control/speed_limiter");

  // get conversion parameters
  speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").get_value<double>();
  speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").get_value<double>();
  steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
  steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();
  cmd_timeout_ = get_parameter("cmd_timeout").get_value<double>();
  vesc_state_timeout_ = get_parameter("vesc_state_timeout").get_value<double>();

  SpeedLimiter::Config cfg;
  cfg.max_accel = get_parameter("max_accel").get_value<double>();
  cfg.max_decel = get_parameter("max_decel").get_value<double>();
  cfg.launch_accel = get_parameter("launch_accel").get_value<double>();
  cfg.launch_blend_speed = get_parameter("launch_blend_speed").get_value<double>();
  cfg.max_speed_lead = get_parameter("max_speed_lead").get_value<double>();
  cfg.grip_accel_limit = get_parameter("grip_accel_limit").get_value<double>();
  cfg.min_corner_accel = get_parameter("min_corner_accel").get_value<double>();
  cfg.wheelbase = get_parameter("wheelbase").get_value<double>();
  cfg.high_steer_angle = get_parameter("high_steer_angle").get_value<double>();
  cfg.high_steer_launch_accel = get_parameter("high_steer_launch_accel").get_value<double>();
  cfg.stall_cmd_min_speed = get_parameter("stall_cmd_min_speed").get_value<double>();
  cfg.stall_measured_max_speed = get_parameter("stall_measured_max_speed").get_value<double>();
  cfg.stall_min_motor_current = get_parameter("stall_min_motor_current").get_value<double>();
  cfg.stall_detect_time = get_parameter("stall_detect_time").get_value<double>();
  cfg.stall_reset_duration = get_parameter("stall_reset_duration").get_value<double>();
  cfg.stall_max_retries = static_cast<int>(get_parameter("stall_max_retries").get_value<int64_t>());

  limiter_ = std::make_unique<SpeedLimiter>(cfg);

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));

  if (limiterEnabled()) {
    RCLCPP_INFO(
      get_logger(),
      "speed limiter: launch=%.2f->max=%.2f m/s^2 (blend %.2f m/s), decel=%.2f m/s^2, "
      "lead_cap=%.2f m/s, grip=%.2f m/s^2, stall_guard=%.2f s, timeout=%.2f s",
      cfg.launch_accel, cfg.max_accel, cfg.launch_blend_speed, cfg.max_decel,
      cfg.max_speed_lead, cfg.grip_accel_limit, cfg.stall_detect_time, cmd_timeout_);
    // measured wheel speed & motor current from VESC for closed-loop lead cap and stall guard
    vesc_state_sub_ = create_subscription<VescStateStamped>(
      "sensors/core", 10, std::bind(&AckermannToVesc::vescStateCallback, this, _1));
    // The limiter runs on its own clock, so a single stop command (the gamepad sends one when manual
    // control ends) is followed all the way down instead of leaving the VESC on the last speed.
    limiter_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&AckermannToVesc::limiterTimerCallback, this));
    // target vs limited speed and TC/stall state for the Drive panel: 10 Hz, best effort (rule 7)
    limiter_status_pub_ = create_publisher<SpeedLimiterStatus>(
      get_parameter("limiter_status_topic").get_value<std::string>(), rclcpp::QoS(1).best_effort());
  }
}

void AckermannToVesc::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  if (speed_to_erpm_gain_ != 0.0) {
    measured_speed_ = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
    motor_current_ = state->state.current_motor;
    last_vesc_time_ = steady_clock_.now();
    have_measured_ = true;
  }
}

bool AckermannToVesc::hasFreshVescState(const rclcpp::Time & now) const
{
  if (!have_measured_) {
    return false;
  }
  if (vesc_state_timeout_ <= 0.0) {
    return true;
  }
  return (now - last_vesc_time_).seconds() <= vesc_state_timeout_;
}

void AckermannToVesc::limiterTimerCallback()
{
  if (++tick_count_ % 5 == 0) {  // status at 10 Hz for the Foxglove Drive panel
    publishLimiterStatus();
  }
  if (!active_) {
    return;
  }
  const rclcpp::Time t = steady_clock_.now();
  const double dt = std::clamp((t - last_tick_time_).seconds(), 0.0, 0.1);
  last_tick_time_ = t;

  // No command for cmd_timeout: brake to a stop at max_decel, then go quiet (the VESC's own
  // timeout then releases the motor).
  const bool stale = (t - last_cmd_time_).seconds() > cmd_timeout_;
  SpeedLimiter::Input in;
  in.target_speed = stale ? 0.0 : target_speed_;
  in.cmd_accel = stale ? 0.0 : cmd_accel_;
  in.steering_angle = steering_angle_;
  in.measured_speed = measured_speed_;
  in.motor_current = motor_current_;
  in.have_measured = hasFreshVescState(t);
  in.stopping = stale;

  const SpeedLimiter::Output out = limiter_->step(in, dt);
  if (out.stall_triggered) {
    RCLCPP_WARN(
      get_logger(),
      "Launch stall detected (v=%.2f m/s, I_motor=%.1f A, retry %u): resetting VESC speed PID",
      measured_speed_, motor_current_, out.stall_count);
  }
  if (out.fault_triggered) {
    RCLCPP_ERROR(
      get_logger(),
      "Launch stall fault latched after %u consecutive stalls (v=%.2f m/s, I_motor=%.1f A): "
      "commanding 0 ERPM until command returns to zero",
      out.stall_count, measured_speed_, motor_current_);
  }

  publishSpeed(out.limited_speed);
  if (stale && out.limited_speed == 0.0) {
    active_ = false;
  }
}

void AckermannToVesc::publishLimiterStatus()
{
  const rclcpp::Time t = steady_clock_.now();
  const bool stale = (t - last_cmd_time_).seconds() > cmd_timeout_;
  const SpeedLimiter::Output & last = limiter_->lastOutput();

  SpeedLimiterStatus status;
  status.header.stamp = now();
  status.target_speed = (active_ && !stale) ? static_cast<float>(target_speed_) : 0.0f;
  status.limited_speed = active_ ? static_cast<float>(limiter_->limitedSpeed()) : 0.0f;
  status.max_accel = static_cast<float>(limiter_->config().max_accel);
  status.max_decel = static_cast<float>(limiter_->config().max_decel);
  status.active = active_;
  status.stopping = active_ && stale;
  status.effective_accel = active_ ? static_cast<float>(last.effective_accel) : 0.0f;
  status.measured_speed = hasFreshVescState(t) ? static_cast<float>(measured_speed_) : 0.0f;
  status.lead_clamped = active_ && last.lead_clamped;
  status.stall_recovering = active_ && last.stall_recovering;
  status.stall_fault = active_ && last.stall_fault;
  status.stall_count = active_ ? last.stall_count : 0u;
  limiter_status_pub_->publish(status);
}

void AckermannToVesc::publishSpeed(double speed)
{
  // calc vesc electric RPM (speed)
  Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * speed + speed_to_erpm_offset_;
  if (rclcpp::ok()) {
    erpm_pub_->publish(erpm_msg);
  }
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  if (limiterEnabled()) {
    const rclcpp::Time t = steady_clock_.now();
    if (!active_) {
      // First command, or the first after a stop: start from the measured speed, so the car
      // neither jumps nor brakes hard if it is still rolling.
      limiter_->seed(hasFreshVescState(t) ? measured_speed_ : 0.0);
      last_tick_time_ = t;
      active_ = true;
    }
    target_speed_ = cmd->drive.speed;
    cmd_accel_ = cmd->drive.acceleration;
    steering_angle_ = cmd->drive.steering_angle;
    last_cmd_time_ = t;
  } else {
    publishSpeed(cmd->drive.speed);
  }

  // calc steering angle (servo)
  Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // publish
  if (rclcpp::ok()) {
    servo_pub_->publish(servo_msg);
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
