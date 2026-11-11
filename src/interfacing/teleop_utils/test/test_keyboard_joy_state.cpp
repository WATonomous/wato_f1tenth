// Copyright (c) 2025-present WATonomous. All rights reserved.
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

#include <cmath>

#include <catch2/catch_all.hpp>

#include "keyboard_joy_state.hpp"  // NOLINT(build/include_subdir)

namespace
{
// joypad_config.yaml's layout: steering axis 0, R2 axis 5, L2 axis 2, X button 0, R1 button 10
constexpr int kSteer = 0, kThrottle = 5, kBrake = 2, kX = 0, kR1 = 10;
constexpr double kMaxSpeed = 22.88;

KeyboardJoyState makeState()
{
  KeyboardJoyConfig config;
  config.steering_axis = kSteer;
  config.throttle_axis = kThrottle;
  config.brake_axis = kBrake;
  config.safety_button = kX;
  config.autonomy_button = kR1;
  config.num_axes = 6;
  config.num_buttons = 11;
  config.max_speed = kMaxSpeed;
  config.steering_rate = 5.0;
  config.speed_levels = {1.0, 2.0, 3.0, 4.0, 5.0};
  config.initial_level = 0;
  config.initial_hold_s = 0.55;
  config.repeat_hold_s = 0.15;
  return KeyboardJoyState(config);
}

// joypad_node's trigger_maper and how it scales the result
double joypadSpeed(float trigger)
{
  const double ct = -(trigger - 1.0) / 2.0;
  return (std::exp(4.0 * (ct - 1.0)) - 0.01832) * kMaxSpeed;
}

// A key held as a terminal sends it: one press, a 0.5 s gap, then a repeat every 30 ms.
// Ticks update() at 20 Hz from start_s to end_s.
void holdKey(KeyboardJoyState & state, char key, double start_s, double end_s)
{
  state.handleKey(key, start_s);
  double next_key = start_s + 0.5;
  for (double t = start_s; t < end_s; t += 0.05) {
    for (; next_key <= t; next_key += 0.03) {
      state.handleKey(key, next_key);
    }
    state.update(t);
  }
}
}  // namespace

TEST_CASE("at rest it looks like an untouched gamepad", "[keyboard_joy]")
{
  auto state = makeState();
  state.update(0.0);
  const auto axes = state.axes();
  const auto buttons = state.buttons();
  REQUIRE(axes.size() == 6);
  REQUIRE(buttons.size() == 11);
  // released triggers rest at +1.0; 0.0 would drive the car
  REQUIRE(axes[kThrottle] == 1.0f);
  REQUIRE(axes[kBrake] == 1.0f);
  REQUIRE(axes[kSteer] == 0.0f);
  REQUIRE(joypadSpeed(axes[kThrottle]) == Catch::Approx(0.0).margin(1e-3));
  for (auto button : buttons) {
    REQUIRE(button == 0);
  }
}

TEST_CASE("space and f toggle X and R1 like sticky buttons", "[keyboard_joy]")
{
  auto state = makeState();
  REQUIRE(state.handleKey(' ', 0.0) == KeyEvent::kChanged);
  REQUIRE(state.buttons()[kX] == 1);
  REQUIRE(state.handleKey('f', 0.0) == KeyEvent::kChanged);
  REQUIRE(state.buttons()[kR1] == 1);
  state.handleKey(' ', 0.1);
  state.handleKey('F', 0.1);
  REQUIRE(state.buttons()[kX] == 0);
  REQUIRE(state.buttons()[kR1] == 0);
}

TEST_CASE("w drives at the speed level through joypad_node's trigger curve", "[keyboard_joy]")
{
  auto state = makeState();
  for (double level : {1.0, 2.0, 3.0, 4.0, 5.0}) {
    INFO("level " << level);
    state.handleKey('w', 0.0);
    state.update(0.0);
    REQUIRE(state.speedLevel() == level);
    REQUIRE(joypadSpeed(state.axes()[kThrottle]) == Catch::Approx(level).epsilon(1e-4));
    state.handleKey('q', 0.0);
  }
}

TEST_CASE("q and e stay within the speed levels", "[keyboard_joy]")
{
  auto state = makeState();
  REQUIRE(state.handleKey('e', 0.0) == KeyEvent::kChanged);
  REQUIRE(state.speedLevel() == 1.0);
  for (int i = 0; i < 10; ++i) {
    state.handleKey('q', 0.0);
  }
  REQUIRE(state.speedLevel() == 5.0);
  state.handleKey('e', 0.0);
  REQUIRE(state.speedLevel() == 4.0);
}

TEST_CASE("throttle lasts only while w is held", "[keyboard_joy]")
{
  auto state = makeState();

  SECTION("held through the repeat delay it never drops out, then stops on release")
  {
    state.handleKey('w', 0.0);
    double next_key = 0.5;
    for (double t = 0.0; t < 2.0; t += 0.05) {
      for (; next_key <= t; next_key += 0.03) {
        state.handleKey('w', next_key);
      }
      state.update(t);
      REQUIRE(state.throttleHeld());
    }
    state.update(2.3);
    REQUIRE_FALSE(state.throttleHeld());
    REQUIRE(state.axes()[kThrottle] == 1.0f);
  }

  SECTION("a single tap lasts initial_hold_s")
  {
    state.handleKey('w', 0.0);
    state.update(0.5);
    REQUIRE(state.throttleHeld());
    state.update(0.6);
    REQUIRE_FALSE(state.throttleHeld());
  }

  SECTION("s brakes fully and cancels w")
  {
    holdKey(state, 'w', 0.0, 0.2);
    state.handleKey('s', 0.2);
    state.update(0.25);
    REQUIRE_FALSE(state.throttleHeld());
    REQUIRE(state.axes()[kBrake] == -1.0f);
    REQUIRE(state.axes()[kThrottle] == 1.0f);
  }
}

TEST_CASE("steering ramps to full lock and recentres on release", "[keyboard_joy]")
{
  auto state = makeState();
  holdKey(state, 'a', 0.0, 1.0);
  REQUIRE(state.axes()[kSteer] == Catch::Approx(1.0));

  // the last repeat is at about 0.98 s, so a counts as released from about 1.13 s
  state.update(1.2);
  REQUIRE(state.steering() < 1.0);
  state.update(1.5);
  REQUIRE(state.steering() == Catch::Approx(0.0).margin(1e-9));

  SECTION("d steers the other way and cancels a")
  {
    holdKey(state, 'a', 2.0, 2.6);
    holdKey(state, 'd', 2.6, 3.5);
    REQUIRE(state.axes()[kSteer] == Catch::Approx(-1.0));
  }
}

TEST_CASE("the other axis stays alive while the newest key repeats", "[keyboard_joy]")
{
  auto state = makeState();
  holdKey(state, 'w', 0.0, 1.0);

  // a pressed with w still down: w stops repeating, a takes over
  holdKey(state, 'a', 1.0, 2.0);
  REQUIRE(state.throttleHeld());
  REQUIRE(state.axes()[kSteer] == Catch::Approx(1.0));

  state.update(2.3);
  REQUIRE_FALSE(state.throttleHeld());
}

TEST_CASE("releaseAll leaves an untouched gamepad", "[keyboard_joy]")
{
  auto state = makeState();
  state.handleKey(' ', 0.0);
  state.handleKey('f', 0.0);
  holdKey(state, 'a', 0.0, 0.5);
  state.handleKey('w', 0.5);
  state.releaseAll();
  const auto axes = state.axes();
  REQUIRE(axes[kThrottle] == 1.0f);
  REQUIRE(axes[kBrake] == 1.0f);
  REQUIRE(axes[kSteer] == 0.0f);
  REQUIRE(state.buttons()[kX] == 0);
  REQUIRE(state.buttons()[kR1] == 0);
}

TEST_CASE("Ctrl-C quits and other keys are ignored", "[keyboard_joy]")
{
  auto state = makeState();
  REQUIRE(state.handleKey(0x03, 0.0) == KeyEvent::kQuit);
  REQUIRE(state.handleKey('m', 0.0) == KeyEvent::kIgnored);
  REQUIRE(state.handleKey('x', 0.0) == KeyEvent::kIgnored);
}
