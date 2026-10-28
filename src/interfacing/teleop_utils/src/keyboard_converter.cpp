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

#include "keyboard_converter.hpp"  // NOLINT(build/include_subdir)

#include <string>

KeyboardConverter::KeyboardConverter()
: Node("keyboard_converter_node")
{
  // parameters
  this->declare_parameter<std::string>("twist_topic", "/cmd_vel");
  this->declare_parameter<std::string>("ackermann_topic", "/hal/drive/keyboard");

  twist_topic = this->get_parameter("twist_topic").as_string();
  ackermann_topic = this->get_parameter("ackermann_topic").as_string();

  // pubs and subs
  ackermann_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_topic, 10);

  twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    twist_topic, 10, std::bind(&KeyboardConverter::converter, this, std::placeholders::_1));
}

void KeyboardConverter::converter(geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  ackermann_msgs::msg::AckermannDriveStamped equev_ackermannn;
  equev_ackermannn.header.frame_id = "base_link";
  equev_ackermannn.header.stamp = this->now();

  if (twist_msg->linear.x != 0) {
    equev_ackermannn.drive.speed = std::clamp(1.0 * twist_msg->linear.x, -3.5, 20.0);
  } else if (twist_msg->angular.z != 0) {
    equev_ackermannn.drive.speed = 0.5;
    equev_ackermannn.drive.steering_angle = std::clamp(M_PI / 10 * twist_msg->angular.z, -0.5236, 0.5236);
  } else {
    equev_ackermannn.drive.speed = 0;
    equev_ackermannn.drive.steering_angle = 0;
  }

  ackermann_pub->publish(equev_ackermannn);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardConverter>());
  rclcpp::shutdown();
  return 0;
}
