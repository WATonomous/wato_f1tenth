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

#include "joypad_teleop.hpp"  // NOLINT(build/include_subdir)

#include <string>

/*
future todo
give it gears for fun and a button to turn off the gear
gears as in ability to shift though gears like a gt car
*/

JOYPAD::JOYPAD()
: Node("joypad_node")
{
  // parameters
  this->declare_parameter<std::string>("gamepad_topic", "/hal/drive/joystick");
  this->declare_parameter<std::string>("joy_topic", "/joy");
  this->declare_parameter<std::string>("dead_man_topic", "/hal/dead_man_switch");
  this->declare_parameter<std::string>("mode_topic", "/control/mode");
  this->declare_parameter<std::string>("my_frame_id", "base_link");

  this->declare_parameter<float>("max_speed", 22.88);
  this->declare_parameter<float>("steering_gain", 0.5236);
  this->declare_parameter<float>("max_steering_rate", 3.2);

  this->declare_parameter<int>("steering_axis", 0);
  this->declare_parameter<int>("throttel_axis", 5);
  this->declare_parameter<int>("break_axis", 2);
  this->declare_parameter<int>("safety_button", 0);
  this->declare_parameter<int>("reverse_button", 4);
  this->declare_parameter<int>("pit_limit_button", 5);
  this->declare_parameter<int>("autonomy_button", 10);
  this->declare_parameter<double>("joy_timeout_s", 0.5);

  gamepad_topic = this->get_parameter("gamepad_topic").as_string();
  joy_topic = this->get_parameter("joy_topic").as_string();
  my_frame_id = this->get_parameter("my_frame_id").as_string();
  dead_man_topic = this->get_parameter("dead_man_topic").as_string();

  max_speed = this->get_parameter("max_speed").as_double();
  max_steering_rate = this->get_parameter("max_steering_rate").as_double();
  steering_gain = this->get_parameter("steering_gain").as_double();

  // button
  ba.steering_axis = this->get_parameter("steering_axis").as_int();
  ba.throttle_axis = this->get_parameter("throttel_axis").as_int();
  ba.break_axis = this->get_parameter("break_axis").as_int();

  ba.safety_button = this->get_parameter("safety_button").as_int();
  ba.reverse_button = this->get_parameter("reverse_button").as_int();
  ba.pitLimit_button = this->get_parameter("pit_limit_button").as_int();
  ba.autonomy_button = this->get_parameter("autonomy_button").as_int();
  joy_timeout_s = this->get_parameter("joy_timeout_s").as_double();

  // pubs and sub
  ackerman_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(gamepad_topic, 10);

  dead_man_pub_ =
    this->create_publisher<std_msgs::msg::Bool>(dead_man_topic, rclcpp::QoS(1).transient_local().reliable());
  // MANUAL, AUTONOMY, BLOCKED, IDLE or NO_GAMEPAD, latched and sent on change (the status panel)
  mode_pub_ = this->create_publisher<std_msgs::msg::String>(
    this->get_parameter("mode_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
  publishMode("NO_GAMEPAD");

  gamepad_sub = this->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic, 20, std::bind(&JOYPAD::gamepadCallback, this, std::placeholders::_1));

  // dead man watchdog: if /joy stops (pad disconnected, joy_node gone) while the autonomy button
  // is held, the latched dead man would stay true and autonomy would keep driving
  watchdog_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&JOYPAD::watchdogCallback, this));

  // emergancy drive message initalization
  emergancy_drive_msg.speed = 0.0;
  emergancy_drive_msg.steering_angle = 0.0;
}

void JOYPAD::publishDeadMan(bool active)
{
  std_msgs::msg::Bool dead_man_bool;
  dead_man_bool.data = active;
  dead_man_pub_->publish(dead_man_bool);
  dead_man_ = active;
}

void JOYPAD::publishMode(const std::string & mode)
{
  if (mode == mode_) {
    return;
  }
  mode_ = mode;
  std_msgs::msg::String msg;
  msg.data = mode;
  mode_pub_->publish(msg);
}

void JOYPAD::watchdogCallback()
{
  if (last_joy_.nanoseconds() > 0 && (this->now() - last_joy_).seconds() > joy_timeout_s) {
    publishMode("NO_GAMEPAD");
  }
  if (dead_man_ && last_joy_.nanoseconds() > 0 && (this->now() - last_joy_).seconds() > joy_timeout_s) {
    RCLCPP_WARN(
      this->get_logger(), "no %s for %.1f s: releasing the dead man switch", joy_topic.c_str(), joy_timeout_s);
    publishDeadMan(false);
  }
}

void JOYPAD::gamepadCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  ackermann_msgs::msg::AckermannDrive drive;
  ackermann_msgs::msg::AckermannDriveStamped drive_msg;

  drive_msg.header.frame_id = my_frame_id;
  drive_msg.header.stamp = msg->header.stamp;

  bool pit_speed_limit = false;
  last_joy_ = this->now();

  // Two buttons, both toggles (joy_node runs with sticky_buttons):
  //   safety_button (X) on: manual control. This node publishes on gamepad_topic, which beats
  //     autonomy in ackermann_mux, and the dead man is released.
  //   autonomy_button (R1) on, X off: the dead man switch; autonomy may drive. Toggling it off
  //     stops the controller.
  // Turning manual on also blocks autonomy until R1 is toggled off and on again, so turning
  // manual off (after using X to stop the car) never lets autonomy drive off by itself.
  bool armed = msg->buttons.at(ba.safety_button);
  bool autonomy_on = msg->buttons.at(ba.autonomy_button);
  if (armed) {
    autonomy_blocked_ = true;
  } else if (!autonomy_on) {
    autonomy_blocked_ = false;
  }
  bool autonomy = !armed && autonomy_on && !autonomy_blocked_;
  publishMode(armed ? "MANUAL" : autonomy ? "AUTONOMY" : (autonomy_on && autonomy_blocked_) ? "BLOCKED" : "IDLE");

  // On the manual->released edge, publish one zero command so the last thing downstream sees is a
  // stop, then go quiet on gamepad_topic. ackermann_mux (timeout: 0.2s in mux.yaml) then sees the
  // topic expire and falls through to autonomy. Publishing zero every message here (instead of
  // once) would keep gamepad_topic fresh forever and permanently lock out autonomy.
  if (!armed) {
    if (was_armed_) {
      drive_msg.drive = emergancy_drive_msg;
      ackerman_pub->publish(drive_msg);
      was_armed_ = false;
    }
    publishDeadMan(autonomy);
    return;
  }
  was_armed_ = true;

  float current_steering = JOYPAD::steering_mapper(msg->axes.at(ba.steering_axis));
  float current_throtel = JOYPAD::trigger_maper(msg->axes.at(ba.throttle_axis));
  float current_break = JOYPAD::trigger_maper(msg->axes.at(ba.break_axis));

  // reversing
  if (msg->buttons.at(ba.reverse_button)) {
    reversing = true;
    direction = -1;
  } else {
    reversing = false;
    direction = 1;
  }

  if (msg->buttons.at(ba.pitLimit_button)) {
    pit_speed_limit = true;
  }

  // throtel logic
  float total_throtel = current_throtel - current_break;
  if (total_throtel < 0.0) {
    total_throtel = 0.0;
  }

  drive.speed = total_throtel * max_speed * direction;
  drive.steering_angle = current_steering;
  drive.steering_angle_velocity = max_steering_rate;

  if (pit_speed_limit && drive.speed > 1.0 && !reversing) {
    drive.speed = 1.0;
  }

  if (reversing && drive.speed < -1.5) {
    drive.speed = -1.5;
  }

  // publish the message; manual control releases the dead man so autonomy stops
  drive_msg.drive = drive;
  ackerman_pub->publish(drive_msg);
  publishDeadMan(false);
}

float JOYPAD::trigger_maper(const float r2)
{
  float ct = -(r2 - 1) / 2;
  return (std::exp(4 * (ct - 1)) - 0.01832);
}

float JOYPAD::steering_mapper(const float ls)
{
  return ls * steering_gain;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JOYPAD>());
  rclcpp::shutdown();
  return 0;
}
