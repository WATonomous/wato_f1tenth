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

#include "keyboard_teleop.hpp"  // NOLINT(build/include_subdir)

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

/*
Sim-only keyboard teleop, read from the terminal it runs in (so it is started by hand, not by
interfacing.launch.yaml, whose nodes have no stdin). It does what the gamepad does:
  - space toggles the dead man (/hal/dead_man_switch), so autonomy may drive
  - m takes over from autonomy on its own mux input (/hal/drive/keyboard_teleop, above autonomy
    and below the joystick in config/sim/mux.yaml), and releases the dead man
It refuses to start unless `enabled` is true, which only config/sim/keyboard_teleop.yaml sets.
*/

KeyboardTeleop::KeyboardTeleop()
: Node("keyboard_teleop_node")
{
  // parameters
  this->declare_parameter<bool>("enabled", false);
  this->declare_parameter<std::string>("drive_topic", "/hal/drive/keyboard_teleop");
  this->declare_parameter<std::string>("dead_man_topic", "/hal/dead_man_switch");
  this->declare_parameter<std::string>("my_frame_id", "base_link");
  this->declare_parameter<double>("publish_rate_hz", 20.0);

  this->declare_parameter<double>("max_speed", 1.0);
  this->declare_parameter<double>("max_reverse_speed", 0.5);
  this->declare_parameter<double>("speed_step", 0.25);
  this->declare_parameter<double>("max_steering", 0.5236);
  this->declare_parameter<double>("steering_step", 0.05);

  if (!this->get_parameter("enabled").as_bool()) {
    throw std::runtime_error(
      "enabled is false: keyboard_teleop is sim only, start it with interfacing_bringup's "
      "config/sim/keyboard_teleop.yaml");
  }
  if (!isatty(STDIN_FILENO)) {
    throw std::runtime_error("stdin is not a terminal: run it in an interactive shell (docker exec -it)");
  }

  drive_topic = this->get_parameter("drive_topic").as_string();
  dead_man_topic = this->get_parameter("dead_man_topic").as_string();
  my_frame_id = this->get_parameter("my_frame_id").as_string();

  KeyboardTeleopLimits limits;
  limits.max_speed = this->get_parameter("max_speed").as_double();
  limits.max_reverse_speed = this->get_parameter("max_reverse_speed").as_double();
  limits.speed_step = this->get_parameter("speed_step").as_double();
  limits.max_steering = this->get_parameter("max_steering").as_double();
  limits.steering_step = this->get_parameter("steering_step").as_double();
  state_.emplace(limits);

  // pubs and subs
  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
  // same QoS as the gamepad's dead man, so both look alike to pure_pursuit and drive_status
  dead_man_pub_ =
    this->create_publisher<std_msgs::msg::Bool>(dead_man_topic, rclcpp::QoS(1).transient_local().reliable());
  // follow the gamepad and Foxglove too, so space toggles what is really there
  dead_man_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    dead_man_topic, rclcpp::QoS(1).reliable(), [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data != state_->deadMan()) {
        state_->syncDeadMan(msg->data);
        printStatus();
      }
    });

  // raw terminal: keys arrive one at a time, no echo, Ctrl-C is a key (so we can stop cleanly)
  termios saved;
  tcgetattr(STDIN_FILENO, &saved);
  saved_termios_ = saved;
  termios raw = saved;
  raw.c_lflag &= ~(ICANON | ECHO | ISIG);
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);

  // stop the car however we exit (q, Ctrl-C, SIGTERM from docker)
  this->get_node_base_interface()->get_context()->add_pre_shutdown_callback([this]() { release(); });

  const double rate_hz = this->get_parameter("publish_rate_hz").as_double();
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_hz), std::bind(&KeyboardTeleop::timerCallback, this));

  std::printf(
    "\nkeyboard teleop (sim only)\n"
    "  space  dead man on/off (autonomy may drive)\n"
    "  m      manual on/off (takes over from autonomy, releases the dead man)\n"
    "  w/s    faster/slower (reverse below 0)   a/d  steer left/right   x  stop\n"
    "  q      quit (stops the car and releases the dead man)\n\n");
  printStatus();
}

KeyboardTeleop::~KeyboardTeleop()
{
  if (saved_termios_) {
    tcsetattr(STDIN_FILENO, TCSANOW, &*saved_termios_);
  }
  std::printf("\n");
}

void KeyboardTeleop::timerCallback()
{
  char buf[32];
  const ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
  for (ssize_t i = 0; i < n; ++i) {
    // arrow and function keys send ESC [ A ...: drop the rest of the burst, or 'A' would steer
    if (buf[i] == 0x1b) {
      break;
    }
    switch (state_->handleKey(buf[i])) {
      case KeyEvent::kDeadMan:
        publishDeadMan(state_->deadMan());
        break;
      case KeyEvent::kManualOn:
        // always, even if we think it is off: the gamepad or Foxglove may have turned it on
        publishDeadMan(false);
        break;
      case KeyEvent::kManualOff:
        // one zero command so the last thing downstream sees is a stop, then go quiet so the mux
        // input times out (0.2 s) and falls through to autonomy, which waits for the dead man
        publishDrive(0.0, 0.0);
        break;
      case KeyEvent::kQuit:
        rclcpp::shutdown();
        return;
      case KeyEvent::kDrive:
      case KeyEvent::kIgnored:
        break;
    }
    printStatus();
  }

  if (state_->manual()) {
    publishDrive(state_->speed(), state_->steering());
  }
}

void KeyboardTeleop::publishDrive(double speed, double steering)
{
  ackermann_msgs::msg::AckermannDriveStamped msg;
  msg.header.frame_id = my_frame_id;
  msg.header.stamp = this->now();
  msg.drive.speed = speed;
  msg.drive.steering_angle = steering;
  drive_pub_->publish(msg);
}

void KeyboardTeleop::publishDeadMan(bool active)
{
  std_msgs::msg::Bool msg;
  msg.data = active;
  dead_man_pub_->publish(msg);
}

void KeyboardTeleop::release()
{
  if (released_) {
    return;
  }
  released_ = true;
  if (state_->manual()) {
    publishDrive(0.0, 0.0);
  }
  if (state_->deadMan()) {
    publishDeadMan(false);
  }
}

void KeyboardTeleop::printStatus()
{
  const char * mode = state_->manual() ? "MANUAL" : state_->deadMan() ? "AUTONOMY" : "IDLE";
  std::printf(
    "\r  %-8s  dead man %-3s  speed %5.2f m/s  steering %5.2f rad\033[K",
    mode,
    state_->deadMan() ? "on" : "off",
    state_->speed(),
    state_->steering());
  std::fflush(stdout);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<KeyboardTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "keyboard_teleop_node: %s\n", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
