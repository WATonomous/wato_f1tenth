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

#include "vesc_ackermann/speed_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vesc_ackermann
{

namespace
{

void applyCap(double & current_cap, double candidate)
{
  if (candidate <= 0.0) {
    return;
  }
  current_cap = (current_cap > 0.0) ? std::min(current_cap, candidate) : candidate;
}

}  // namespace

SpeedLimiter::SpeedLimiter(const Config & config)
: cfg_(config)
{
}

void SpeedLimiter::seed(double speed)
{
  limited_speed_ = speed;
  stall_elapsed_ = 0.0;
  reset_remaining_ = 0.0;
  stall_count_ = 0;
  stall_fault_ = false;
  last_output_ = Output{};
  last_output_.limited_speed = speed;
}

double SpeedLimiter::computeEffectiveAccel(const Input & in) const
{
  double eff = cfg_.max_accel > 0.0 ? cfg_.max_accel : 0.0;
  const double v_ref = in.have_measured ? std::abs(in.measured_speed) : std::abs(limited_speed_);

  // 1. Speed-scheduled launch acceleration
  if (cfg_.launch_accel > 0.0 && cfg_.launch_blend_speed > 0.0) {
    const double alpha = std::clamp(v_ref / cfg_.launch_blend_speed, 0.0, 1.0);
    const double top = cfg_.max_accel > 0.0 ? cfg_.max_accel : cfg_.launch_accel;
    const double scheduled = cfg_.launch_accel + alpha * (top - cfg_.launch_accel);
    applyCap(eff, scheduled);
  }

  // 2. Low-speed high-steering launch cap (reduces wheelspin when launching off-line / turned)
  if (cfg_.high_steer_launch_accel > 0.0 && cfg_.high_steer_angle > 0.0) {
    const bool in_launch_zone =
      (cfg_.launch_blend_speed <= 0.0) || (v_ref < cfg_.launch_blend_speed);
    if (in_launch_zone && std::abs(in.steering_angle) >= cfg_.high_steer_angle) {
      applyCap(eff, cfg_.high_steer_launch_accel);
    }
  }

  // 3. Friction-circle lateral-acceleration derating
  if (cfg_.grip_accel_limit > 0.0 && cfg_.wheelbase > 0.0) {
    // Clamp steering angle away from pi/2 to avoid tan() overflow
    const double abs_steer = std::min(std::abs(in.steering_angle), 1.4);
    const double ay = (v_ref * v_ref * std::tan(abs_steer)) / cfg_.wheelbase;
    const double grip_sq = cfg_.grip_accel_limit * cfg_.grip_accel_limit;
    const double ax_avail = std::sqrt(std::max(0.0, grip_sq - ay * ay));
    const double floor_accel = std::max(0.0, cfg_.min_corner_accel);
    applyCap(eff, std::max(floor_accel, ax_avail));
  }

  // 4. Honor AckermannDrive.acceleration > 0 when supplied by a planner/controller
  if (in.cmd_accel > 0.0) {
    applyCap(eff, in.cmd_accel);
  }

  return eff;
}

double SpeedLimiter::rampTowards(
  double current, double target, double accel_limit, double dt) const
{
  if (target == current) {
    return target;
  }
  // Moving away from zero is speeding up (accel_limit); towards zero or reversing is slowing down
  // (max_decel). The VESC speed controller brakes to follow a falling command.
  const bool speeding_up =
    (current >= 0.0 && target > current) || (current <= 0.0 && target < current);
  const double rate = speeding_up ? accel_limit : cfg_.max_decel;
  if (rate <= 0.0) {
    return target;
  }
  const double step = rate * dt;
  double next =
    target > current ? std::min(target, current + step) : std::max(target, current - step);
  // Stop at zero before changing direction, then speed up the other way at accel_limit.
  if ((current > 0.0 && next < 0.0) || (current < 0.0 && next > 0.0)) {
    next = 0.0;
  }
  return next;
}

SpeedLimiter::Output SpeedLimiter::step(const Input & in, double dt)
{
  const double safe_dt = std::max(0.0, dt);
  Output out{};
  const double target = in.stopping ? 0.0 : in.target_speed;
  const bool zero_cmd = std::abs(target) < 1e-6;

  // Releasing the command (zero target or timeout) clears any latched stall state so the next
  // deliberate launch starts fresh.
  if (zero_cmd) {
    stall_elapsed_ = 0.0;
    reset_remaining_ = 0.0;
    stall_count_ = 0;
    stall_fault_ = false;
  }

  // Latched stall fault holds 0 until target returns to 0.
  if (stall_fault_) {
    limited_speed_ = 0.0;
    out.limited_speed = 0.0;
    out.effective_accel = 0.0;
    out.stall_fault = true;
    out.stall_count = static_cast<uint8_t>(std::min(stall_count_, 255));
    last_output_ = out;
    return out;
  }

  // Active 0-ERPM integrator reset pause after a detected stall.
  if (reset_remaining_ > 0.0) {
    reset_remaining_ = std::max(0.0, reset_remaining_ - safe_dt);
    limited_speed_ = 0.0;
    out.limited_speed = 0.0;
    out.effective_accel = 0.0;
    out.stall_recovering = true;
    out.stall_count = static_cast<uint8_t>(std::min(stall_count_, 255));
    last_output_ = out;
    return out;
  }

  // Stall watchdog: detects sensorless open-loop hang at ~0 ERPM with high motor current.
  if (cfg_.stall_detect_time > 0.0 && in.have_measured && !in.stopping) {
    if (std::abs(in.measured_speed) > cfg_.stall_measured_max_speed * 2.0) {
      stall_count_ = 0;
    }
    const bool stalled_now =
      std::abs(target) >= cfg_.stall_cmd_min_speed &&
      std::abs(in.measured_speed) <= cfg_.stall_measured_max_speed &&
      std::abs(in.motor_current) >= cfg_.stall_min_motor_current;

    if (stalled_now) {
      stall_elapsed_ += safe_dt;
      if (stall_elapsed_ >= cfg_.stall_detect_time) {
        stall_elapsed_ = 0.0;
        ++stall_count_;
        limited_speed_ = 0.0;
        out.limited_speed = 0.0;
        out.effective_accel = 0.0;
        out.stall_count = static_cast<uint8_t>(std::min(stall_count_, 255));
        if (cfg_.stall_max_retries > 0 && stall_count_ >= cfg_.stall_max_retries) {
          stall_fault_ = true;
          out.stall_fault = true;
          out.fault_triggered = true;
        } else {
          reset_remaining_ = std::max(0.0, cfg_.stall_reset_duration);
          out.stall_recovering = true;
          out.stall_triggered = true;
        }
        last_output_ = out;
        return out;
      }
    } else {
      stall_elapsed_ = 0.0;
    }
  } else {
    stall_elapsed_ = 0.0;
  }

  const double eff_accel = computeEffectiveAccel(in);
  double next = rampTowards(limited_speed_, target, eff_accel, safe_dt);

  // Closed-loop lead cap: never let the ramped speed run far ahead of measured wheel speed while
  // accelerating away from zero (prevents VESC speed PID windup on launch).
  bool lead_clamped = false;
  if (cfg_.max_speed_lead > 0.0 && in.have_measured && !in.stopping) {
    const bool accel_fwd = target > 0.0 && next > 0.0 && target >= limited_speed_;
    const bool accel_rev = target < 0.0 && next < 0.0 && target <= limited_speed_;
    if (accel_fwd) {
      const double cap = std::max(0.0, in.measured_speed) + cfg_.max_speed_lead;
      if (next > cap) {
        next = cap;
        lead_clamped = true;
      }
    } else if (accel_rev) {
      const double cap = std::min(0.0, in.measured_speed) - cfg_.max_speed_lead;
      if (next < cap) {
        next = cap;
        lead_clamped = true;
      }
    }
  }

  limited_speed_ = next;
  out.limited_speed = limited_speed_;
  out.effective_accel = eff_accel;
  out.lead_clamped = lead_clamped;
  out.stall_recovering = false;
  out.stall_fault = false;
  out.stall_count = static_cast<uint8_t>(std::min(stall_count_, 255));
  last_output_ = out;
  return out;
}

}  // namespace vesc_ackermann
