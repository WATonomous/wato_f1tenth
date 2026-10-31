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

#ifndef KEYBOARD_TELEOP
#define KEYBOARD_TELEOP

#include <termios.h>

#include <memory>
#include <optional>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "keyboard_teleop_state.hpp"  // NOLINT(build/include_subdir)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop();
  ~KeyboardTeleop() override;

private:
  // pubs and subs
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dead_man_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dead_man_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // functions
  void timerCallback();
  void publishDrive(double speed, double steering);
  void publishDeadMan(bool active);
  void release();
  void printStatus();

  // parameters
  std::string drive_topic, dead_man_topic, my_frame_id;

  // data
  std::optional<KeyboardTeleopState> state_;
  std::optional<termios> saved_termios_;
  bool released_ = false;
};

#endif
