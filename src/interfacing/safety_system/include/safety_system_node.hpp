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

#ifndef SAFETY_SYSTEM_NODE_HPP_
#define SAFETY_SYSTEM_NODE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"

struct Parameters
{
  int alarm_threshold, debounce_threshold;
  double TTC_stage1, TTC_stage2, TTC_stage3, WHEEL_RADIUS, TTC1_Throtel, TTC2_Throtel, TTC3_Throtel;
  double alpha, encoder_bias, lp_factor;  // must be between 0 and 1
  double velocity_threshold, acell_threshold;
};

class SafetyNode : public rclcpp::Node
{
public:
  // funtions
  SafetyNode();

  // call back
  void laiderCallBack(const sensor_msgs::msg::LaserScan::SharedPtr laider_msg);
  void imuCallBack(const sensor_msgs::msg::Imu::SharedPtr data);
  void findEncoderVelocity();

private:
  // publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laider_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_encoder_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_encoder_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throtel_monitor_sub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throtel_input_pub;

  // data
  sensor_msgs::msg::JointState::SharedPtr left_encoder_data;
  sensor_msgs::msg::JointState::SharedPtr right_encoder_data;

  rclcpp::Time last_imu_time, last_encoder_time;
  double imu_velocity, encoder_velocity;
  double encoder_last_rad, last_accl;
  std_msgs::msg::Float32 curret_throtel;
  int debounce_counter = 0;

  // parameters
  Parameters pram;
};
#endif
