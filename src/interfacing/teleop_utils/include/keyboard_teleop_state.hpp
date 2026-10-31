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

#ifndef KEYBOARD_TELEOP_STATE
#define KEYBOARD_TELEOP_STATE

#include <algorithm>
#include <cctype>

// Key handling for keyboard_teleop_node, with no ROS in it so it can be unit tested.
// Same rules as the gamepad (joypad_teleop.cpp): manual control releases the dead man, and
// leaving manual never hands the car back to autonomy until the dead man is toggled on again.

struct KeyboardTeleopLimits
{
  double max_speed, max_reverse_speed, speed_step, max_steering, steering_step;
};

enum class KeyEvent
{
  kIgnored,  // not a key we use, or a drive key while manual is off
  kDeadMan,  // dead man toggled: publish deadMan()
  kManualOn,  // took over: publish the dead man released, then drive at the publish rate
  kManualOff,  // gave control back: publish one zero command, then go quiet
  kDrive,  // speed or steering setpoint changed (manual only)
  kQuit,
};

class KeyboardTeleopState
{
public:
  explicit KeyboardTeleopState(const KeyboardTeleopLimits & limits)
  : limits_(limits)
  {}

  // space: dead man on/off (autonomy), m: manual on/off, w/s: speed, a/d: steering,
  // x: stop, q or Ctrl-C: quit
  KeyEvent handleKey(char key)
  {
    switch (std::tolower(static_cast<unsigned char>(key))) {
      case ' ':
        if (manual_) {
          return KeyEvent::kIgnored;  // leave manual first, as with the gamepad's X
        }
        dead_man_ = !dead_man_;
        return KeyEvent::kDeadMan;
      case 'm':
        manual_ = !manual_;
        speed_ = 0.0;
        steering_ = 0.0;
        if (manual_) {
          dead_man_ = false;
          return KeyEvent::kManualOn;
        }
        return KeyEvent::kManualOff;
      case 'q':
      case 0x03:  // Ctrl-C (the terminal is raw, so it arrives as a key)
        return KeyEvent::kQuit;
    }
    if (!manual_) {
      return KeyEvent::kIgnored;
    }
    switch (std::tolower(static_cast<unsigned char>(key))) {
      case 'w':
        speed_ = std::min(speed_ + limits_.speed_step, limits_.max_speed);
        return KeyEvent::kDrive;
      case 's':
        speed_ = std::max(speed_ - limits_.speed_step, -limits_.max_reverse_speed);
        return KeyEvent::kDrive;
      case 'a':  // positive steering is left (REP 103)
        steering_ = std::min(steering_ + limits_.steering_step, limits_.max_steering);
        return KeyEvent::kDrive;
      case 'd':
        steering_ = std::max(steering_ - limits_.steering_step, -limits_.max_steering);
        return KeyEvent::kDrive;
      case 'x':
        speed_ = 0.0;
        steering_ = 0.0;
        return KeyEvent::kDrive;
    }
    return KeyEvent::kIgnored;
  }

  // Another source (the gamepad, Foxglove) changed the dead man: toggle from what is really there
  void syncDeadMan(bool active)
  {
    dead_man_ = active;
  }

  bool manual() const
  {
    return manual_;
  }

  bool deadMan() const
  {
    return dead_man_;
  }

  double speed() const
  {
    return speed_;
  }

  double steering() const
  {
    return steering_;
  }

private:
  KeyboardTeleopLimits limits_;
  bool manual_ = false;
  bool dead_man_ = false;
  double speed_ = 0.0;
  double steering_ = 0.0;
};

#endif
