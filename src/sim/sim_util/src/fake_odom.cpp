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

#include "fake_odom.hpp"  // NOLINT(build/include_subdir)

#include <memory>
#include <string>

FakeOdom::FakeOdom()
: Node("fake_odom")
{
  // frames declaration
  this->declare_parameter<std::string>("child_frame", "base_link");
  this->declare_parameter<std::string>("header_frame", "odom");
  this->declare_parameter<double>("inital_speed", 0.0);
  // AutoDRIVE publishes the car's speed in the twist of its odometry topic
  this->declare_parameter<std::string>("speed_odom_topic", "/autodrive/roboracer_1/odom");
  this->declare_parameter<std::string>("odom_topic", "/hal/odom");
  // Ground-truth localization (localization:=truth): the same pose in the map frame,
  // valid because map -> odom is a static identity then
  this->declare_parameter<bool>("publish_localization", false);
  this->declare_parameter<std::string>("localization_topic", "/hal/localization/odom");
  this->declare_parameter<std::string>("localization_frame", "map");

  child_frame_name = this->get_parameter("child_frame").as_string();
  header_frame_name = this->get_parameter("header_frame").as_string();
  localization_frame_name = this->get_parameter("localization_frame").as_string();
  current_speed.data = static_cast<float>(this->get_parameter("inital_speed").as_double());

  // publisher
  odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(this->get_parameter("odom_topic").as_string(), 10);
  if (this->get_parameter("publish_localization").as_bool()) {
    localization_pub =
      this->create_publisher<nav_msgs::msg::Odometry>(this->get_parameter("localization_topic").as_string(), 10);
  }

  // subscriptions
  tf_sub = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10, std::bind(&FakeOdom::tf_listener, this, std::placeholders::_1));

  speed_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    this->get_parameter("speed_odom_topic").as_string(), 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      current_speed.data = static_cast<float>(msg->twist.twist.linear.x);
    });

  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  RCLCPP_INFO(this->get_logger(), "init the constructor for Fake odom");
}

// listen to the messages
void FakeOdom::tf_listener(const tf2_msgs::msg::TFMessage::SharedPtr tf_msg)
{
  for (const auto & t : tf_msg->transforms) {
    if (t.child_frame_id == "roboracer_1" && t.header.frame_id == "world") {
      nav_msgs::msg::Odometry tf_odom;

      tf_odom.pose.pose.position.x = t.transform.translation.x;
      tf_odom.pose.pose.position.y = t.transform.translation.y;
      tf_odom.pose.pose.position.z = t.transform.translation.z;

      tf_odom.pose.pose.orientation.w = t.transform.rotation.w;
      tf_odom.pose.pose.orientation.x = t.transform.rotation.x;
      tf_odom.pose.pose.orientation.y = t.transform.rotation.y;
      tf_odom.pose.pose.orientation.z = t.transform.rotation.z;

      tf_odom.header.stamp = t.header.stamp;

      FakeOdom::publish_msgs(tf_odom);

      break;
    }
  }
}

// transmit the tf into a odom topic and create a odom -> base_link tf
void FakeOdom::publish_msgs(const nav_msgs::msg::Odometry & odom_msg)
{
  // publishing odom
  nav_msgs::msg::Odometry current_odom;

  current_odom.header.frame_id = header_frame_name;
  current_odom.child_frame_id = child_frame_name;
  current_odom.header.stamp = odom_msg.header.stamp;

  current_odom.pose.pose.position = odom_msg.pose.pose.position;
  current_odom.pose.pose.orientation = odom_msg.pose.pose.orientation;
  current_odom.twist.twist.linear.x = current_speed.data;

  odom_pub->publish(current_odom);

  if (localization_pub) {
    nav_msgs::msg::Odometry localization = current_odom;
    localization.header.frame_id = localization_frame_name;
    localization_pub->publish(localization);
  }

  // publishing tf
  geometry_msgs::msg::TransformStamped t;

  t.child_frame_id = child_frame_name;
  t.header.frame_id = header_frame_name;
  t.header.stamp = odom_msg.header.stamp;

  t.transform.translation.x = odom_msg.pose.pose.position.x;
  t.transform.translation.y = odom_msg.pose.pose.position.y;
  t.transform.translation.z = odom_msg.pose.pose.position.z;

  t.transform.rotation = odom_msg.pose.pose.orientation;

  tf_broadcaster->sendTransform(t);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeOdom>());
  rclcpp::shutdown();
  return 0;
}
