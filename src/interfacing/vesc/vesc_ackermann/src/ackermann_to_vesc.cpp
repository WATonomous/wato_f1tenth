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
#include <sstream>
#include <string>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options)
{
  // declare parameters
  declare_parameter("speed_to_erpm_gain", 0.0);
  declare_parameter("speed_to_erpm_offset", 0.0);
  declare_parameter("steering_angle_to_servo_gain", 0.0);
  declare_parameter("steering_angle_to_servo_offset", 0.0);
  // speed limiter, off unless set in YAML (config/car/vesc.yaml)
  declare_parameter("max_accel", 0.0);
  declare_parameter("max_decel", 0.0);
  declare_parameter("cmd_timeout", 0.5);

  // get conversion parameters
  speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").get_value<double>();
  speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").get_value<double>();
  steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
  steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();
  max_accel_ = get_parameter("max_accel").get_value<double>();
  max_decel_ = get_parameter("max_decel").get_value<double>();
  cmd_timeout_ = get_parameter("cmd_timeout").get_value<double>();

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));

  if (limiterEnabled()) {
    RCLCPP_INFO(
      get_logger(),
      "speed limiter: speed up at %.2f m/s^2, slow down at %.2f m/s^2 (0 = no limit), "
      "stop after %.2f s without a command", max_accel_, max_decel_, cmd_timeout_);
    // measured wheel speed, to start the limiter from when commands begin again
    vesc_state_sub_ = create_subscription<VescStateStamped>(
      "sensors/core", 10, std::bind(&AckermannToVesc::vescStateCallback, this, _1));
    // The limiter runs on its own clock, so a single stop command (the gamepad sends one when manual
    // control ends) is followed all the way down instead of leaving the VESC on the last speed.
    limiter_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&AckermannToVesc::limiterTimerCallback, this));
  }
}

void AckermannToVesc::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  if (speed_to_erpm_gain_ != 0.0) {
    measured_speed_ = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
    have_measured_ = true;
  }
}

double AckermannToVesc::limitSpeed(double current, double target, double dt) const
{
  if (target == current) {
    return target;
  }
  // Moving away from zero is speeding up (max_accel); towards zero, or reversing, is slowing down
  // (max_decel). The VESC speed controller brakes to follow a falling command.
  const bool speeding_up = (current >= 0.0 && target > current) || (current <= 0.0 && target < current);
  const double rate = speeding_up ? max_accel_ : max_decel_;
  if (rate <= 0.0) {
    return target;
  }
  const double step = rate * dt;
  double next = target > current ? std::min(target, current + step) : std::max(target, current - step);
  // Stop at zero before changing direction, then speed up the other way at max_accel.
  if ((current > 0.0 && next < 0.0) || (current < 0.0 && next > 0.0)) {
    next = 0.0;
  }
  return next;
}

void AckermannToVesc::limiterTimerCallback()
{
  if (!active_) {
    return;
  }
  const rclcpp::Time t = steady_clock_.now();
  const double dt = std::clamp((t - last_tick_time_).seconds(), 0.0, 0.1);
  last_tick_time_ = t;

  // No command for cmd_timeout: brake to a stop at max_decel, then go quiet (the VESC's own
  // timeout then releases the motor).
  const bool stale = (t - last_cmd_time_).seconds() > cmd_timeout_;
  const double target = stale ? 0.0 : target_speed_;
  limited_speed_ = limitSpeed(limited_speed_, target, dt);
  publishSpeed(limited_speed_);
  if (stale && limited_speed_ == 0.0) {
    active_ = false;
  }
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
      limited_speed_ = have_measured_ ? measured_speed_ : 0.0;
      last_tick_time_ = t;
      active_ = true;
    }
    target_speed_ = cmd->drive.speed;
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
