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
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <f1tenth_msgs/msg/speed_limiter_status.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;
using f1tenth_msgs::msg::SpeedLimiterStatus;

class AckermannToVesc : public rclcpp::Node
{
public:
  explicit AckermannToVesc(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;

  // speed limiter: m/s^2 when speeding up / slowing down (0 = no limit), and how long (s) without a
  // command before the limiter brakes to a stop
  double max_accel_, max_decel_, cmd_timeout_;

  /** @todo consider also providing an interpolated look-up table conversion */

  // ROS services
  rclcpp::Publisher<Float64>::SharedPtr erpm_pub_;
  rclcpp::Publisher<Float64>::SharedPtr servo_pub_;
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::TimerBase::SharedPtr limiter_timer_;
  rclcpp::Publisher<SpeedLimiterStatus>::SharedPtr limiter_status_pub_;

  // speed limiter state
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  double target_speed_ = 0.0;   // latest commanded speed (m/s)
  double limited_speed_ = 0.0;  // speed sent to the VESC (m/s)
  double measured_speed_ = 0.0;
  bool have_measured_ = false;
  unsigned int tick_count_ = 0;
  bool active_ = false;  // publishing speed; false before the first command and after a stop
  rclcpp::Time last_cmd_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_tick_time_{0, 0, RCL_STEADY_TIME};

  // ROS callbacks
  void ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd);
  void vescStateCallback(const VescStateStamped::SharedPtr state);
  void limiterTimerCallback();

  bool limiterEnabled() const {return max_accel_ > 0.0 || max_decel_ > 0.0;}
  void publishSpeed(double speed);
  void publishLimiterStatus();
  // step the limited speed towards target over dt seconds
  double limitSpeed(double current, double target, double dt) const;
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
