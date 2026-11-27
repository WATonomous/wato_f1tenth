// Copyright 2026 WATonomous
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_ACKERMANN__SPEED_LIMITER_HPP_
#define VESC_ACKERMANN__SPEED_LIMITER_HPP_

#include <cstdint>

namespace vesc_ackermann
{

// Pure-C++ speed limiter, launch stall guard, and feedforward traction control for ackermann_to_vesc.
// Kept ROS-free so unit tests can exercise every path directly.
class SpeedLimiter
{
public:
  struct Config
  {
    // Base ramp limits (m/s^2; <= 0 disables that direction)
    double max_accel = 0.0;
    double max_decel = 0.0;

    // Speed-scheduled launch acceleration: blends linearly from launch_accel at 0 m/s to max_accel
    // at launch_blend_speed (<= 0 disables).
    double launch_accel = 0.0;
    double launch_blend_speed = 0.0;

    // Closed-loop lead cap (m/s): while accelerating away from zero with fresh wheel-speed
    // telemetry, limited_speed stays within measured_speed + max_speed_lead (<= 0 disables).
    double max_speed_lead = 0.0;

    // Friction-circle lateral-accel derating: a_y = v^2 * tan(|delta|) / wheelbase,
    // a_x_avail = max(min_corner_accel, sqrt(max(0, grip_accel_limit^2 - a_y^2))).
    // Disabled when grip_accel_limit <= 0 or wheelbase <= 0.
    double grip_accel_limit = 0.0;
    double min_corner_accel = 0.0;
    double wheelbase = 0.0;

    // Extra low-speed accel cap when |steering_angle| >= high_steer_angle (rad) below
    // launch_blend_speed (<= 0 disables).
    double high_steer_angle = 0.0;
    double high_steer_launch_accel = 0.0;

    // Sensorless launch-stall watchdog: if commanded >= stall_cmd_min_speed while wheel speed
    // stays <= stall_measured_max_speed and |motor_current| >= stall_min_motor_current for
    // stall_detect_time (s), briefly command 0 m/s for stall_reset_duration (s) so the VESC hits
    // s_pid_min_erpm and clears m_speed_i_term. Disabled when stall_detect_time <= 0.
    double stall_cmd_min_speed = 0.3;
    double stall_measured_max_speed = 0.05;
    double stall_min_motor_current = 20.0;
    double stall_detect_time = 0.0;
    double stall_reset_duration = 0.15;
    int stall_max_retries = 3;
  };

  struct Input
  {
    double target_speed = 0.0;
    double cmd_accel = 0.0;       // AckermannDrive.acceleration (> 0 caps effective accel)
    double steering_angle = 0.0;  // latest commanded steering angle (rad)
    double measured_speed = 0.0;  // VESC wheel speed (m/s)
    double motor_current = 0.0;   // VESC motor current (A)
    bool have_measured = false;   // fresh sensors/core telemetry available
    bool stopping = false;        // command stream timed out; braking to 0
  };

  struct Output
  {
    double limited_speed = 0.0;
    double effective_accel = 0.0;
    bool lead_clamped = false;
    bool stall_recovering = false;
    bool stall_fault = false;
    bool stall_triggered = false;  // true on the single tick a stall reset begins
    bool fault_triggered = false;  // true on the single tick stall_fault latches
    uint8_t stall_count = 0;
  };

  explicit SpeedLimiter(const Config & config);

  bool enabled() const
  {
    return cfg_.max_accel > 0.0 || cfg_.max_decel > 0.0 || cfg_.launch_accel > 0.0;
  }

  // Seed limited_speed (e.g. from measured speed on first command after idle) and clear stall state.
  void seed(double speed);

  // Advance limiter by dt seconds and return the speed to send to the VESC plus diagnostic flags.
  Output step(const Input & in, double dt);

  double limitedSpeed() const {return limited_speed_;}
  const Output & lastOutput() const {return last_output_;}
  const Config & config() const {return cfg_;}

private:
  double computeEffectiveAccel(const Input & in) const;
  double rampTowards(double current, double target, double accel_limit, double dt) const;

  Config cfg_;
  double limited_speed_ = 0.0;
  double stall_elapsed_ = 0.0;
  double reset_remaining_ = 0.0;
  int stall_count_ = 0;
  bool stall_fault_ = false;
  Output last_output_{};
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__SPEED_LIMITER_HPP_
