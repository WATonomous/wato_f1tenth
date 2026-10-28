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

#ifndef KEYBOARD_CONVERTER
#define KEYBOARD_CONVERTER

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class KeyboardConverter : public rclcpp::Node
{
public:
  KeyboardConverter();

private:
  // subs and pubs
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub;

  // helper function
  void converter(geometry_msgs::msg::Twist::SharedPtr twist_msg);

  // parameter values
  double top_speed, max_angle;
  std::string twist_topic, ackermann_topic;
};

#endif
