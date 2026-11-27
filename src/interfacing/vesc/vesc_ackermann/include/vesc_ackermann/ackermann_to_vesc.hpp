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

#ifndef VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
#define VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <f1tenth_msgs/msg/speed_limiter_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <memory>

#include "vesc_ackermann/speed_limiter.hpp"

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using f1tenth_msgs::msg::SpeedLimiterStatus;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

class AckermannToVesc : public rclcpp::Node
{
public:
  explicit AckermannToVesc(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;
  double cmd_timeout_;
  double vesc_state_timeout_;

  std::unique_ptr<SpeedLimiter> limiter_;

  // ROS interfaces
  rclcpp::Publisher<Float64>::SharedPtr erpm_pub_;
  rclcpp::Publisher<Float64>::SharedPtr servo_pub_;
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::TimerBase::SharedPtr limiter_timer_;
  rclcpp::Publisher<SpeedLimiterStatus>::SharedPtr limiter_status_pub_;

  // speed limiter runtime state
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  double target_speed_ = 0.0;      // latest commanded speed (m/s)
  double cmd_accel_ = 0.0;         // latest commanded acceleration (m/s^2, 0 = unspecified)
  double steering_angle_ = 0.0;    // latest commanded steering angle (rad)
  double measured_speed_ = 0.0;    // wheel speed from VESC sensors/core (m/s)
  double motor_current_ = 0.0;     // motor current from VESC sensors/core (A)
  bool have_measured_ = false;
  unsigned int tick_count_ = 0;
  bool active_ = false;  // publishing speed; false before the first command and after a stop
  rclcpp::Time last_cmd_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_tick_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_vesc_time_{0, 0, RCL_STEADY_TIME};

  // ROS callbacks
  void ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd);
  void vescStateCallback(const VescStateStamped::SharedPtr state);
  void limiterTimerCallback();

  bool limiterEnabled() const {return limiter_ && limiter_->enabled();}
  bool hasFreshVescState(const rclcpp::Time & now) const;
  void publishSpeed(double speed);
  void publishLimiterStatus();
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
