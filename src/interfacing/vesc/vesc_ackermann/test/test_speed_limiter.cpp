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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "vesc_ackermann/speed_limiter.hpp"

using Catch::Approx;
using vesc_ackermann::SpeedLimiter;

namespace
{

SpeedLimiter::Config makeCarConfig()
{
  SpeedLimiter::Config cfg;
  cfg.max_accel = 3.3;
  cfg.max_decel = 4.0;
  cfg.launch_accel = 2.0;
  cfg.launch_blend_speed = 1.5;
  cfg.max_speed_lead = 0.5;
  cfg.grip_accel_limit = 4.5;
  cfg.min_corner_accel = 0.5;
  cfg.wheelbase = 0.3302;
  cfg.high_steer_angle = 0.25;
  cfg.high_steer_launch_accel = 1.5;
  cfg.stall_cmd_min_speed = 0.3;
  cfg.stall_measured_max_speed = 0.05;
  cfg.stall_min_motor_current = 20.0;
  cfg.stall_detect_time = 0.5;
  cfg.stall_reset_duration = 0.15;
  cfg.stall_max_retries = 3;
  return cfg;
}

}  // namespace

TEST_CASE("Basic acceleration, deceleration, and zero-crossing", "[speed_limiter]")
{
  SpeedLimiter::Config cfg;
  cfg.max_accel = 3.0;
  cfg.max_decel = 4.0;
  SpeedLimiter limiter(cfg);
  limiter.seed(0.0);

  SpeedLimiter::Input in;
  in.target_speed = 5.0;
  auto out = limiter.step(in, 0.1);
  REQUIRE(out.limited_speed == Approx(0.3));
  REQUIRE(out.effective_accel == Approx(3.0));

  // Braking towards zero uses max_decel
  limiter.seed(2.0);
  in.target_speed = -1.0;
  out = limiter.step(in, 0.6);  // 2.0 - 4.0*0.6 = -0.4 -> stops at 0.0 first
  REQUIRE(out.limited_speed == Approx(0.0));

  // Next tick accelerates into reverse using max_accel
  out = limiter.step(in, 0.1);
  REQUIRE(out.limited_speed == Approx(-0.3));
}

TEST_CASE("Closed-loop lead cap prevents launch windup when wheels lag", "[speed_limiter]")
{
  SpeedLimiter::Config cfg;
  cfg.max_accel = 3.3;
  cfg.max_decel = 4.0;
  cfg.max_speed_lead = 0.5;
  SpeedLimiter limiter(cfg);
  limiter.seed(0.0);

  SpeedLimiter::Input in;
  in.target_speed = 3.4;
  in.measured_speed = 0.0;
  in.have_measured = true;

  // Simulate 0.4 s of wheel stall (Nov 21 crash scenario): without lead cap limited_speed reached
  // 1.32 m/s; with lead cap it must stay <= 0.5 m/s.
  SpeedLimiter::Output out;
  for (int i = 0; i < 20; ++i) {
    out = limiter.step(in, 0.02);
  }
  REQUIRE(out.limited_speed == Approx(0.5));
  REQUIRE(out.lead_clamped);

  // Once the wheel starts turning, the cap moves up with measured_speed.
  in.measured_speed = 0.4;
  out = limiter.step(in, 0.02);
  REQUIRE(out.limited_speed == Approx(0.5 + 3.3 * 0.02));
  REQUIRE_FALSE(out.lead_clamped);
}

TEST_CASE("Lead cap falls back cleanly when VESC telemetry is stale", "[speed_limiter]")
{
  SpeedLimiter::Config cfg;
  cfg.max_accel = 3.0;
  cfg.max_decel = 4.0;
  cfg.max_speed_lead = 0.5;
  SpeedLimiter limiter(cfg);
  limiter.seed(0.0);

  SpeedLimiter::Input in;
  in.target_speed = 3.0;
  in.measured_speed = 0.0;
  in.have_measured = false;  // stale telemetry

  SpeedLimiter::Output out;
  for (int i = 0; i < 20; ++i) {
    out = limiter.step(in, 0.02);
  }
  REQUIRE(out.limited_speed == Approx(1.2));
  REQUIRE_FALSE(out.lead_clamped);
}

TEST_CASE("Stall watchdog resets VESC PID, retries, latches fault, and clears on zero cmd",
  "[speed_limiter]")
{
  auto cfg = makeCarConfig();
  SpeedLimiter limiter(cfg);
  limiter.seed(0.0);

  SpeedLimiter::Input in;
  in.target_speed = 2.0;
  in.measured_speed = 0.0;
  in.motor_current = 30.0;
  in.have_measured = true;

  // 24 ticks * 0.02 s = 0.48 s -> not yet triggered
  SpeedLimiter::Output out;
  for (int i = 0; i < 24; ++i) {
    out = limiter.step(in, 0.02);
    REQUIRE_FALSE(out.stall_recovering);
    REQUIRE_FALSE(out.stall_fault);
  }

  // 25th tick hits 0.50 s -> triggers stall #1 and holds 0 m/s for 0.15 s
  out = limiter.step(in, 0.02);
  REQUIRE(out.stall_triggered);
  REQUIRE(out.stall_recovering);
  REQUIRE_FALSE(out.stall_fault);
  REQUIRE(out.stall_count == 1);
  REQUIRE(out.limited_speed == Approx(0.0));

  // During the 0.15 s reset window, limited_speed remains 0.0
  for (int i = 0; i < 8; ++i) {
    out = limiter.step(in, 0.02);
    REQUIRE(out.limited_speed == Approx(0.0));
  }

  // Next tick resumes launching gently from 0.0
  out = limiter.step(in, 0.02);
  REQUIRE_FALSE(out.stall_recovering);
  REQUIRE(out.limited_speed > 0.0);

  // Trigger stall #2 and stall #3 -> stall_fault latches on retry 3
  for (int attempt = 2; attempt <= 3; ++attempt) {
    bool triggered = false;
    for (int i = 0; i < 40; ++i) {
      out = limiter.step(in, 0.02);
      if (out.stall_triggered || out.fault_triggered) {
        triggered = true;
        break;
      }
    }
    REQUIRE(triggered);
    if (attempt < 3) {
      REQUIRE(out.stall_recovering);
      // Finish reset pause
      for (int i = 0; i < 8; ++i) {
        out = limiter.step(in, 0.02);
      }
    }
  }
  REQUIRE(out.stall_fault);
  REQUIRE(out.fault_triggered);
  REQUIRE(out.stall_count == 3);
  REQUIRE(out.limited_speed == Approx(0.0));

  // Remains latched at 0 while target stays non-zero
  out = limiter.step(in, 0.02);
  REQUIRE(out.stall_fault);
  REQUIRE(out.limited_speed == Approx(0.0));

  // Zero command clears the fault latch
  in.target_speed = 0.0;
  in.motor_current = 0.0;
  out = limiter.step(in, 0.02);
  REQUIRE_FALSE(out.stall_fault);
  REQUIRE(out.stall_count == 0);

  // Subsequent launch works again
  in.target_speed = 2.0;
  out = limiter.step(in, 0.02);
  REQUIRE_FALSE(out.stall_fault);
  REQUIRE(out.limited_speed > 0.0);
}

TEST_CASE("Speed-scheduled launch accel blends linearly up to max_accel", "[speed_limiter]")
{
  SpeedLimiter::Config cfg;
  cfg.max_accel = 4.0;
  cfg.max_decel = 4.0;
  cfg.launch_accel = 2.0;
  cfg.launch_blend_speed = 2.0;
  SpeedLimiter limiter(cfg);
  limiter.seed(0.0);

  SpeedLimiter::Input in;
  in.target_speed = 8.0;
  in.have_measured = true;

  in.measured_speed = 0.0;
  auto out0 = limiter.step(in, 0.02);
  REQUIRE(out0.effective_accel == Approx(2.0));

  in.measured_speed = 1.0;  // halfway through blend -> 3.0 m/s^2
  auto out1 = limiter.step(in, 0.02);
  REQUIRE(out1.effective_accel == Approx(3.0));

  in.measured_speed = 3.0;  // past blend -> full max_accel 4.0 m/s^2
  auto out2 = limiter.step(in, 0.02);
  REQUIRE(out2.effective_accel == Approx(4.0));
}

TEST_CASE("Friction-circle lateral derating and high-steer launch cap", "[speed_limiter]")
{
  auto cfg = makeCarConfig();
  SpeedLimiter limiter(cfg);

  // Low speed + large steering angle -> high_steer_launch_accel (1.5 m/s^2)
  limiter.seed(0.2);
  SpeedLimiter::Input in;
  in.target_speed = 5.0;
  in.measured_speed = 0.2;
  in.have_measured = true;
  in.steering_angle = 0.30;  // >= 0.25 rad
  auto low_turn = limiter.step(in, 0.02);
  REQUIRE(low_turn.effective_accel == Approx(1.5));

  // Cornering at 3.0 m/s with 0.15 rad steering:
  // ay = 9.0 * tan(0.15) / 0.3302 = 4.119 m/s^2 -> ax_avail = sqrt(4.5^2 - ay^2) ~= 1.812 m/s^2
  limiter.seed(3.0);
  in.measured_speed = 3.0;
  in.steering_angle = 0.15;
  auto mid_turn = limiter.step(in, 0.02);
  const double expected_ay = 9.0 * std::tan(0.15) / 0.3302;
  const double expected_ax = std::sqrt(4.5 * 4.5 - expected_ay * expected_ay);
  REQUIRE(mid_turn.effective_accel == Approx(expected_ax).margin(1e-3));

  // Very hard cornering where ay > grip_accel_limit -> clamps cleanly to min_corner_accel (0.5)
  limiter.seed(5.0);
  in.measured_speed = 5.0;
  in.steering_angle = 0.25;
  auto hard_turn = limiter.step(in, 0.02);
  REQUIRE(hard_turn.effective_accel == Approx(0.5));
}

TEST_CASE("AckermannDrive.acceleration caps effective accel when positive and is ignored at 0",
  "[speed_limiter]")
{
  SpeedLimiter::Config cfg;
  cfg.max_accel = 3.3;
  cfg.max_decel = 4.0;
  SpeedLimiter limiter(cfg);
  limiter.seed(0.0);

  SpeedLimiter::Input in;
  in.target_speed = 5.0;
  in.cmd_accel = 1.2;
  auto capped = limiter.step(in, 0.1);
  REQUIRE(capped.effective_accel == Approx(1.2));
  REQUIRE(capped.limited_speed == Approx(0.12));

  // Higher cmd_accel than max_accel cannot exceed max_accel
  in.cmd_accel = 10.0;
  auto limited = limiter.step(in, 0.1);
  REQUIRE(limited.effective_accel == Approx(3.3));

  // 0.0 means unspecified -> uses max_accel
  in.cmd_accel = 0.0;
  auto default_accel = limiter.step(in, 0.1);
  REQUIRE(default_accel.effective_accel == Approx(3.3));
}
