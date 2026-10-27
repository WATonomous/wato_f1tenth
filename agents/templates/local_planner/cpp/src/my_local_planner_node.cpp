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

// Local planner node for action_bringup (LOCAL_PLANNER=my_local_planner); the interface is in
// agents/workflows/add-local-planner.md. You normally only edit plan() and Params.
//
// Plans at plan_hz. Publishes /local_planner/path while plan() returns a path, and holds the
// latched /local_planner/active flag true only then. With stale inputs or no path the flag goes
// false and the controller falls back to the global raceline.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

#include "my_local_planner/my_local_planner.hpp"

namespace my_local_planner
{

class MyLocalPlannerNode : public rclcpp::Node
{
public:
  MyLocalPlannerNode()
  : Node("my_local_planner_node")
  {
    // Topics follow the contract (contract.yaml); the values live in config/{car,sim}
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/hal/localization/odom");
    const auto speed_topic = declare_parameter<std::string>("speed_topic", "/hal/odom");
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/hal/scan");
    const auto global_path_topic = declare_parameter<std::string>("global_path_topic", "/global_planner/path");
    const auto path_topic = declare_parameter<std::string>("path_topic", "/local_planner/path");
    const auto active_topic = declare_parameter<std::string>("active_topic", "/local_planner/active");
    const double plan_hz = declare_parameter<double>("plan_hz", 20.0);
    stale_s_ = declare_parameter<double>("stale_s", 0.5);

    params_.horizon_m = declare_parameter<double>("horizon_m", params_.horizon_m);

    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, rclcpp::QoS(1).reliable());
    active_pub_ = create_publisher<std_msgs::msg::Bool>(active_topic, latched);

    odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & q = msg->pose.pose.orientation;
        state_.x = msg->pose.pose.position.x;
        state_.y = msg->pose.pose.position.y;
        state_.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        last_odom_ = now();
      });
    speed_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      speed_topic, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        state_.speed = msg->twist.twist.linear.x;
      });
    // SensorDataQoS (best effort) matches both reliable and best-effort scan publishers
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        scan_ = *msg;
        last_scan_ = now();
      });
    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic, latched, [this](nav_msgs::msg::Path::ConstSharedPtr msg) { global_path_ = *msg; });

    set_active(false);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / plan_hz), [this]() { on_timer(); });
  }

private:
  void on_timer()
  {
    const auto t = now();
    const bool fresh = last_odom_.nanoseconds() > 0 && last_scan_.nanoseconds() > 0 &&
                       (t - last_odom_).seconds() < stale_s_ && (t - last_scan_).seconds() < stale_s_;
    if (!fresh || global_path_.poses.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for localization, scan and the global path");
      set_active(false);
      return;
    }

    auto path = plan(state_, scan_, global_path_, params_);
    if (!path || path->poses.empty()) {
      set_active(false);
      return;
    }
    path->header.stamp = t;
    path->header.frame_id = "base_link";
    path_pub_->publish(*path);
    set_active(true);
  }

  // Latched, so only send changes
  void set_active(bool active)
  {
    if (active_ && *active_ == active) {
      return;
    }
    active_ = active;
    std_msgs::msg::Bool msg;
    msg.data = active;
    active_pub_->publish(msg);
  }

  Params params_;
  State state_;
  sensor_msgs::msg::LaserScan scan_;
  nav_msgs::msg::Path global_path_;
  std::optional<bool> active_;
  double stale_s_ = 0.5;
  rclcpp::Time last_odom_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace my_local_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_local_planner::MyLocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
