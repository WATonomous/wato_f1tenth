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

#ifndef FAKE_ODOM
#define FAKE_ODOM

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class FakeOdom : public rclcpp::Node
{
public:
  FakeOdom();

private:
  // publisher and subscribers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub;

  // tf2 broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // helper functions
  void tf_listener(const tf2_msgs::msg::TFMessage::SharedPtr tf_msgs);
  void publish_msgs(const nav_msgs::msg::Odometry & odom_msg);

  // frame names
  std::string child_frame_name;
  std::string header_frame_name;

  // data
  std_msgs::msg::Float32 current_speed;
};

#endif
