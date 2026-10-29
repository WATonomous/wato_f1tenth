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

#ifndef JOYPAD_NODE
#define JOYPAD_NODE

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

struct button_and_axis
{
  int throttle_axis, steering_axis, break_axis, safety_button, reverse_button, pitLimit_button, autonomy_button;
};

class JOYPAD : public rclcpp::Node
{
public:
  JOYPAD();

private:
  // pubs and subs
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_sub;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackerman_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dead_man_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;

  // functions
  void gamepadCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void publishDeadMan(bool active);
  void publishMode(const std::string & mode);
  void watchdogCallback();
  float trigger_maper(const float r2);
  float steering_mapper(const float ls);

  // parameters
  std::string joy_topic, gamepad_topic, dead_man_topic, my_frame_id;
  float max_speed, max_steering_rate, steering_gain;
  double joy_timeout_s;
  ackermann_msgs::msg::AckermannDrive emergancy_drive_msg;

  // data
  float direction = 1;
  bool reversing = false;
  bool was_armed_ = false;
  bool dead_man_ = false;
  bool autonomy_blocked_ = false;
  std::string mode_;
  rclcpp::Time last_joy_{0, 0, RCL_ROS_TIME};

  // buttons
  button_and_axis ba;
};

#endif
