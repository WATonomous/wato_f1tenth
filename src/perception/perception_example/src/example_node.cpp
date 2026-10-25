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

#include "perception_example/example_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

namespace perception_example
{

ExampleNode::ExampleNode(const rclcpp::NodeOptions & options)
: LifecycleNode("example_node", options)
{
  this->declare_parameter<double>("publish_rate_hz", 1.0);

  RCLCPP_INFO(this->get_logger(), "ExampleNode created");
}

ExampleNode::CallbackReturn ExampleNode::on_configure(const rclcpp_lifecycle::State &)
{
  publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "publish_rate_hz must be > 0");
    return CallbackReturn::FAILURE;
  }

  pub_ = this->create_publisher<std_msgs::msg::String>("heartbeat", 10);

  RCLCPP_INFO(this->get_logger(), "Configured: publish_rate_hz=%.1f", publish_rate_hz_);
  return CallbackReturn::SUCCESS;
}

ExampleNode::CallbackReturn ExampleNode::on_activate(const rclcpp_lifecycle::State &)
{
  auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_hz_));

  timer_ = this->create_wall_timer(period_ns, std::bind(&ExampleNode::timer_callback, this));

  pub_->on_activate();

  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

ExampleNode::CallbackReturn ExampleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_->on_deactivate();
  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

ExampleNode::CallbackReturn ExampleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_.reset();
  RCLCPP_INFO(this->get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

ExampleNode::CallbackReturn ExampleNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_.reset();
  RCLCPP_INFO(this->get_logger(), "Shut down");
  return CallbackReturn::SUCCESS;
}

void ExampleNode::timer_callback()
{
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "perception alive " + std::to_string(count_++);
  pub_->publish(std::move(msg));
}

}  // namespace perception_example

RCLCPP_COMPONENTS_REGISTER_NODE(perception_example::ExampleNode)
