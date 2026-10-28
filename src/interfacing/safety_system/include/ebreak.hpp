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

#ifndef EBREAK_NODE_HPP_
#define EBREAK_NODE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"

class EBREAK_NODE : public rclcpp::Node
{
public:
  EBREAK_NODE();

private:
  // pubs
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub;

  // subs
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_mon;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_steering_mon;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  // debug (can't have this active during race or actual testing)
  // rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub;

  // functions
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void laser_callbackv2(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  // data
  double current_speed;
  ackermann_msgs::msg::AckermannDriveStamped current_ackermann_msg, current_steering;
  bool is_breaking = false;
  int prock_counter = 0;
  float desired_speed;

  // parameters
  std::string ackermann_output_topic, ackermann_mon_topic, laser_topic, odom_topic, steering_topic;
  double ttc1, ttc2, ttc3;
  double ttc_throtel_1, ttc_throtel_2, ttc_throtel_3;
  double speed_threshold, look_ofset;
  double MAX_SPEED;
  int alarm_threshold;
};

#endif
