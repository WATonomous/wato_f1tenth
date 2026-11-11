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

#ifndef KEYBOARD_JOY_STATE
#define KEYBOARD_JOY_STATE

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

// Keys -> the sensor_msgs/Joy a gamepad would send, for keyboard_joy_node. No ROS in it, so it can be
// unit tested. joypad_node turns the result into drive commands exactly as it does for a real pad:
//
//   w / s    R2 / L2 (throttle / brake) while held
//   a / d    left stick (steer left / right) while held
//   q / e    raise / lower the speed level (what full w drives at)
//   space    X: manual on/off (sticky, like joy_node's sticky_buttons)
//   f        R1: dead man on/off for autonomy (sticky)
//   Ctrl-C   quit
//
// A terminal sends no key-up, only repeats (about 500 ms after the press, then every 30 ms), and only
// for the newest key. So a key counts as held until its repeats stop: initial_hold_s after the first
// press, repeat_hold_s after each repeat. Pressing a key on the other axis keeps the held one alive
// (w + a: only a repeats, but the car must keep accelerating through the corner).

struct KeyboardJoyConfig
{
  int steering_axis, throttle_axis, brake_axis, safety_button, autonomy_button;
  std::size_t num_axes, num_buttons;
  double max_speed;  // joypad_node's: full trigger drives at max_speed
  double steering_rate;  // left stick units per second
  std::vector<double> speed_levels;  // m/s, ascending
  std::size_t initial_level;  // index into speed_levels
  double initial_hold_s, repeat_hold_s;
};

enum class KeyEvent
{
  kIgnored,  // not a key we use
  kChanged,
  kQuit,
};

class KeyboardJoyState
{
public:
  explicit KeyboardJoyState(const KeyboardJoyConfig & config)
  : config_(config)
  , level_(config.initial_level)
  {}

  // now_s: any clock in seconds, the same one update() gets
  KeyEvent handleKey(char key, double now_s)
  {
    switch (std::tolower(static_cast<unsigned char>(key))) {
      case 'w':
        press(throttle_, brake_, steer_left_, steer_right_, now_s);
        return KeyEvent::kChanged;
      case 's':
        press(brake_, throttle_, steer_left_, steer_right_, now_s);
        return KeyEvent::kChanged;
      case 'a':  // left stick left is +1 under joy_node, and positive steering is left
        press(steer_left_, steer_right_, throttle_, brake_, now_s);
        return KeyEvent::kChanged;
      case 'd':
        press(steer_right_, steer_left_, throttle_, brake_, now_s);
        return KeyEvent::kChanged;
      case 'q':
        level_ = std::min(level_ + 1, config_.speed_levels.size() - 1);
        return KeyEvent::kChanged;
      case 'e':
        level_ = level_ > 0 ? level_ - 1 : 0;
        return KeyEvent::kChanged;
      case ' ':
        safety_ = !safety_;
        return KeyEvent::kChanged;
      case 'f':
        autonomy_ = !autonomy_;
        return KeyEvent::kChanged;
      case 0x03:  // Ctrl-C (the terminal is raw, so it arrives as a key)
        return KeyEvent::kQuit;
    }
    return KeyEvent::kIgnored;
  }

  // Once per publish tick: drop keys whose repeats stopped and move the stick toward its target
  void update(double now_s)
  {
    const double dt = last_update_s_ < 0.0 ? 0.0 : now_s - last_update_s_;
    last_update_s_ = now_s;
    for (HeldKey * key : {&throttle_, &brake_, &steer_left_, &steer_right_}) {
      if (now_s >= key->until_s) {
        key->held = false;
      }
    }
    const double target = (steer_left_.held ? 1.0 : 0.0) - (steer_right_.held ? 1.0 : 0.0);
    const double step = config_.steering_rate * dt;
    steering_ += std::clamp(target - steering_, -step, step);
  }

  // Everything released and both toggles off: the last /joy before quitting, so joypad_node stops
  // the car and drops the dead man straight away instead of waiting for its watchdog
  void releaseAll()
  {
    throttle_ = brake_ = steer_left_ = steer_right_ = HeldKey{};
    steering_ = 0.0;
    safety_ = autonomy_ = false;
  }

  std::vector<float> axes() const
  {
    std::vector<float> axes(config_.num_axes, 0.0f);
    axes[config_.steering_axis] = static_cast<float>(steering_);
    axes[config_.throttle_axis] =
      static_cast<float>(throttle_.held ? triggerFor(speedLevel() / config_.max_speed) : kTriggerReleased);
    axes[config_.brake_axis] = static_cast<float>(brake_.held ? kTriggerPressed : kTriggerReleased);
    return axes;
  }

  std::vector<int32_t> buttons() const
  {
    std::vector<int32_t> buttons(config_.num_buttons, 0);
    buttons[config_.safety_button] = safety_ ? 1 : 0;
    buttons[config_.autonomy_button] = autonomy_ ? 1 : 0;
    return buttons;
  }

  // Trigger position that joypad_node's trigger_maper turns into `fraction` of max_speed. A released
  // trigger rests at +1.0 (0 throttle); 0.0 would already be about 12% throttle.
  static double triggerFor(double fraction)
  {
    if (fraction <= 0.0) {
      return kTriggerReleased;
    }
    const double ct = std::clamp(1.0 + std::log(fraction + kTriggerOffset) / 4.0, 0.0, 1.0);
    return 1.0 - 2.0 * ct;
  }

  // The fastest level joypad_node's trigger curve can reach (full trigger is ~98% of max_speed)
  static double maxFraction()
  {
    return 1.0 - kTriggerOffset;
  }

  double speedLevel() const
  {
    return config_.speed_levels[level_];
  }

  bool throttleHeld() const
  {
    return throttle_.held;
  }

  bool brakeHeld() const
  {
    return brake_.held;
  }

  double steering() const
  {
    return steering_;
  }

  bool safety() const
  {
    return safety_;
  }

  bool autonomy() const
  {
    return autonomy_;
  }

private:
  static constexpr double kTriggerReleased = 1.0;
  static constexpr double kTriggerPressed = -1.0;
  static constexpr double kTriggerOffset = 0.01832;  // joypad_node's trigger_maper

  struct HeldKey
  {
    bool held = false;
    double until_s = 0.0;
  };

  // A key on the same axis cancels the opposite one (no left + right); keys on the other axis stay
  // alive as long as this one, because the terminal now repeats only this key
  void press(HeldKey & key, HeldKey & opposite, HeldKey & other1, HeldKey & other2, double now_s)
  {
    const bool repeat = key.held && now_s < key.until_s;
    key.held = true;
    key.until_s = now_s + (repeat ? config_.repeat_hold_s : config_.initial_hold_s);
    opposite = HeldKey{};
    for (HeldKey * other : {&other1, &other2}) {
      if (other->held) {
        other->until_s = std::max(other->until_s, key.until_s);
      }
    }
  }

  KeyboardJoyConfig config_;
  std::size_t level_;
  HeldKey throttle_, brake_, steer_left_, steer_right_;
  double steering_ = 0.0;
  double last_update_s_ = -1.0;
  bool safety_ = false;
  bool autonomy_ = false;
};

#endif
