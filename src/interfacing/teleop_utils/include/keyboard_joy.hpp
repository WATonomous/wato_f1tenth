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

#ifndef KEYBOARD_JOY
#define KEYBOARD_JOY

#include <termios.h>

#include <optional>
#include <string>

#include "keyboard_joy_state.hpp"  // NOLINT(build/include_subdir)
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "terminal_link.hpp"  // NOLINT(build/include_subdir)

class KeyboardJoy : public rclcpp::Node
{
public:
  KeyboardJoy();
  ~KeyboardJoy() override;

private:
  // pubs and subs
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // functions
  void timerCallback();
  bool readKeys(double now_s);
  void waitForTerminal(double timeout_s);
  void sendStatusQuery();
  void publishJoy();
  void printStatus();

  // parameters
  std::string joy_topic, my_frame_id;

  // data
  std::optional<KeyboardJoyState> state_;
  TerminalInput input_;
  std::optional<LinkMonitor> link_;
  bool link_lost_ = false;
  std::string note_;  // shown in the status line until note_until_s_
  double note_until_s_ = 0.0;
  std::optional<termios> saved_termios_;
  std::string mode_ = "UNKNOWN";  // joypad_node's /control/mode
  std::string last_status_;
};

#endif
