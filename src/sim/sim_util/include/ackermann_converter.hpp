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

#ifndef CONVERTER
#define CONVERTER

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class AckermannConverter : public rclcpp::Node
{
public:
  AckermannConverter();

private:
  // pubs and sub
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackerman_input_sub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throtel_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub;

  // callbacks
  void ackermannCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

  // parameters
  std::string throtel_topic, steering_topic, input_topic;
  double max_speed, steering_gain;
};

#endif
