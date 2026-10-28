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

// Controller node for action_bringup (CONTROLLER=my_controller); the interface is in
// agents/workflows/add-controller.md. You normally only edit compute() and Params.
//
// Runs one control step per localization update (rate-limited to control_hz), so control never
// waits on a timer. Publishes a stop when the dead man switch is released, when there is no
// path, and from a watchdog when localization goes stale.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "my_controller/my_controller.hpp"

namespace my_controller
{

class MyControllerNode : public rclcpp::Node
{
public:
  MyControllerNode()
  : Node("my_controller_node")
  {
    // Topics follow the contract (contract.yaml); the values live in config/{car,sim}
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/hal/localization/odom");
    const auto speed_topic = declare_parameter<std::string>("speed_topic", "/hal/odom");
    const auto global_path_topic = declare_parameter<std::string>("global_path_topic", "/global_planner/path");
    const auto local_path_topic = declare_parameter<std::string>("local_path_topic", "/local_planner/path");
    const auto local_active_topic = declare_parameter<std::string>("local_active_topic", "/local_planner/active");
    const auto dead_man_topic = declare_parameter<std::string>("dead_man_topic", "/hal/dead_man_switch");
    const auto drive_topic = declare_parameter<std::string>("drive_topic", "/hal/drive/autonomy");
    const auto debug_topic = declare_parameter<std::string>("debug_topic", "/debug/my_controller/target");
    // Set by the launch file: true when LOCAL_PLANNER is not none
    use_local_planner_ = declare_parameter<bool>("use_local_planner", false);
    control_period_ = 1.0 / declare_parameter<double>("control_hz", 50.0);
    stale_s_ = declare_parameter<double>("stale_s", 0.5);
    enable_debug_vis_ = declare_parameter<bool>("enable_debug_vis", false);

    params_.max_speed = declare_parameter<double>("max_speed", params_.max_speed);
    params_.max_steering_angle = declare_parameter<double>("max_steering_angle", params_.max_steering_angle);
    params_.wheelbase = declare_parameter<double>("wheelbase", params_.wheelbase);

    // Control topics stay RELIABLE; latched topics (global path, local "active" flag) need
    // transient_local to receive the value published before this node started
    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    debug_pub_ = create_publisher<visualization_msgs::msg::Marker>(debug_topic, rclcpp::QoS(1).best_effort());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_odom(*msg); });
    speed_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      speed_topic, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        state_.speed = msg->twist.twist.linear.x;
      });
    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic, latched, [this](nav_msgs::msg::Path::ConstSharedPtr msg) { global_path_ = *msg; });
    // Volatile + reliable matches the gamepad's latched publisher and plain ones such as Foxglove
    dead_man_sub_ = create_subscription<std_msgs::msg::Bool>(
      dead_man_topic, rclcpp::QoS(1).reliable(), [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        dead_man_ = msg->data;
      });
    if (use_local_planner_) {
      local_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        local_path_topic, rclcpp::QoS(1).reliable(), [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
          local_path_ = *msg;
          local_path_rx_ = now();
        });
      local_active_sub_ = create_subscription<std_msgs::msg::Bool>(
        local_active_topic, latched, [this](std_msgs::msg::Bool::ConstSharedPtr msg) { local_active_ = msg->data; });
    }

    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { on_watchdog(); });
  }

private:
  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto t = now();
    last_odom_ = t;
    if (last_step_.nanoseconds() > 0 && (t - last_step_).seconds() < control_period_) {
      return;
    }
    last_step_ = t;

    const auto & q = msg.pose.pose.orientation;
    state_.x = msg.pose.pose.position.x;
    state_.y = msg.pose.pose.position.y;
    state_.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    if (!dead_man_) {
      publish_stop();
      return;
    }

    // Follow the local planner's path while it is active and fresh, else the global raceline
    const bool local =
      use_local_planner_ && local_active_ && !local_path_.poses.empty() && (t - local_path_rx_).seconds() < stale_s_;
    const auto & path = local ? local_path_ : global_path_;
    if (path.poses.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no path yet, holding the car stopped");
      publish_stop();
      return;
    }

    const Command cmd = compute(state_, path, local, params_);
    publish(
      std::clamp(cmd.steering_angle, -params_.max_steering_angle, params_.max_steering_angle),
      std::clamp(cmd.speed, 0.0, params_.max_speed));
    if (enable_debug_vis_ && cmd.debug_x && cmd.debug_y) {
      publish_debug(path.header.frame_id, *cmd.debug_x, *cmd.debug_y);
    }
  }

  void on_watchdog()
  {
    if (last_odom_.nanoseconds() == 0 || (now() - last_odom_).seconds() > stale_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no localization, holding the car stopped");
      publish_stop();
    }
  }

  void publish_stop()
  {
    publish(0.0, 0.0);
  }

  void publish(double steering_angle, double speed)
  {
    ackermann_msgs::msg::AckermannDriveStamped out;
    out.header.stamp = now();
    out.header.frame_id = "base_link";
    out.drive.steering_angle = steering_angle;
    out.drive.speed = speed;
    drive_pub_->publish(out);
  }

  void publish_debug(const std::string & frame_id, double x, double y)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = now();
    m.header.frame_id = frame_id;
    m.ns = "my_controller";
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.2;
    m.color.r = 1.0;
    m.color.a = 1.0;
    debug_pub_->publish(m);
  }

  Params params_;
  State state_;
  nav_msgs::msg::Path global_path_;
  nav_msgs::msg::Path local_path_;
  bool use_local_planner_ = false;
  bool local_active_ = false;
  bool dead_man_ = false;
  bool enable_debug_vis_ = false;
  double control_period_ = 0.02;
  double stale_s_ = 0.5;
  rclcpp::Time last_odom_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_step_{0, 0, RCL_ROS_TIME};
  rclcpp::Time local_path_rx_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr local_active_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dead_man_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace my_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_controller::MyControllerNode>());
  rclcpp::shutdown();
  return 0;
}
