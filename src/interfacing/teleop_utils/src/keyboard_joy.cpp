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

#include "keyboard_joy.hpp"  // NOLINT(build/include_subdir)

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

/*
Keyboard stand-in for joy_node (TELEOP=keyboard): reads keys from the terminal it runs in and
publishes the sensor_msgs/Joy a gamepad would, so joypad_node stays the only teleop logic (mux input,
dead man, /control/mode, BLOCKED, watchdog). Started by `watod teleop`, which runs it with
docker exec -it, also over SSH on the car. It loads joypad_config.yaml (button and axis layout,
max_speed, steering), keyboard_config.yaml (speed levels, key hold times, link check) and the
platform's interfacing_bringup/config/<platform>/keyboard_teleop.yaml (allow_autonomy).

Link check: it asks the terminal for its status every link_ping_period_s, and every common terminal
answers by itself. No answer for link_timeout_s (an SSH link that dropped without the car noticing)
means everything is released at once: X and R1 off, so joypad_node stops the car and drops the dead
man. They stay off when the link comes back. If the terminal closes, the node quits and /joy stops,
so joypad_node's watchdog releases the dead man.
*/

KeyboardJoy::KeyboardJoy()
: Node("keyboard_joy_node")
{
  // joypad_config.yaml: the same names and defaults as joypad_node
  this->declare_parameter<std::string>("joy_topic", "/joy");
  this->declare_parameter<std::string>("my_frame_id", "base_link");
  this->declare_parameter<double>("max_speed", 4.0);
  this->declare_parameter<double>("steering_gain", 0.5236);
  this->declare_parameter<double>("max_steering_rate", 3.2);
  this->declare_parameter<int>("steering_axis", 0);
  this->declare_parameter<int>("throttel_axis", 5);
  this->declare_parameter<int>("break_axis", 2);
  this->declare_parameter<int>("safety_button", 0);
  this->declare_parameter<int>("reverse_button", 4);
  this->declare_parameter<int>("pit_limit_button", 5);
  this->declare_parameter<int>("autonomy_button", 10);

  // keyboard_config.yaml
  this->declare_parameter<std::string>("mode_topic", "/control/mode");
  this->declare_parameter<double>("publish_rate_hz", 20.0);
  this->declare_parameter<std::vector<double>>("speed_levels", std::vector<double>{1.0});
  this->declare_parameter<double>("initial_speed_level", 1.0);
  this->declare_parameter<double>("initial_hold_s", 0.55);
  this->declare_parameter<double>("repeat_hold_s", 0.15);
  this->declare_parameter<double>("link_ping_period_s", 0.25);
  this->declare_parameter<double>("link_timeout_s", 1.0);
  this->declare_parameter<double>("link_startup_timeout_s", 2.0);

  // interfacing_bringup/config/<platform>/keyboard_teleop.yaml: may the keyboard arm autonomy (R1)?
  this->declare_parameter<bool>("allow_autonomy", false);

  if (!isatty(STDIN_FILENO)) {
    throw std::runtime_error("stdin is not a terminal: start it with `watod teleop`");
  }

  joy_topic = this->get_parameter("joy_topic").as_string();
  my_frame_id = this->get_parameter("my_frame_id").as_string();

  KeyboardJoyConfig config;
  config.steering_axis = this->get_parameter("steering_axis").as_int();
  config.throttle_axis = this->get_parameter("throttel_axis").as_int();
  config.brake_axis = this->get_parameter("break_axis").as_int();
  config.safety_button = this->get_parameter("safety_button").as_int();
  config.autonomy_button = this->get_parameter("autonomy_button").as_int();
  config.reverse_button = this->get_parameter("reverse_button").as_int();
  config.pit_limit_button = this->get_parameter("pit_limit_button").as_int();
  // as many axes and buttons as joypad_node reads, including the ones the keyboard leaves at rest
  config.num_axes = 1 + std::max({config.steering_axis, config.throttle_axis, config.brake_axis});
  config.num_buttons =
    1 + std::max({config.safety_button, config.autonomy_button, config.reverse_button, config.pit_limit_button});
  config.max_speed = this->get_parameter("max_speed").as_double();
  config.steering_rate =
    this->get_parameter("max_steering_rate").as_double() / this->get_parameter("steering_gain").as_double();
  config.speed_levels = this->get_parameter("speed_levels").as_double_array();
  config.initial_hold_s = this->get_parameter("initial_hold_s").as_double();
  config.repeat_hold_s = this->get_parameter("repeat_hold_s").as_double();
  config.allow_autonomy = this->get_parameter("allow_autonomy").as_bool();

  std::sort(config.speed_levels.begin(), config.speed_levels.end());
  if (config.speed_levels.back() > config.max_speed * KeyboardJoyState::maxFraction()) {
    throw std::runtime_error("speed_levels: the top level is above what full throttle reaches (max_speed)");
  }
  const double initial = this->get_parameter("initial_speed_level").as_double();
  const auto found = std::find(config.speed_levels.begin(), config.speed_levels.end(), initial);
  if (found == config.speed_levels.end()) {
    throw std::runtime_error("initial_speed_level is not one of speed_levels");
  }
  config.initial_level = static_cast<std::size_t>(found - config.speed_levels.begin());
  state_.emplace(config);

  // pubs and subs
  joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic, 10);
  // joypad_node's mode, latched: MANUAL, AUTONOMY, BLOCKED, IDLE or NO_GAMEPAD
  mode_sub_ = this->create_subscription<std_msgs::msg::String>(
    this->get_parameter("mode_topic").as_string(),
    rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      mode_ = msg->data;
      printStatus();
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

  link_.emplace(
    this->get_parameter("link_ping_period_s").as_double(), this->get_parameter("link_timeout_s").as_double());
  waitForTerminal(this->get_parameter("link_startup_timeout_s").as_double());

  // On Ctrl-C, or SIGINT / SIGTERM (watod teleop stopping it): one last /joy with everything released.
  // A hang-up (SIGHUP) or kill -9 is not caught: the node just dies, /joy stops, and joypad_node's
  // 0.5 s watchdog releases the dead man.
  this->get_node_base_interface()->get_context()->add_pre_shutdown_callback([this]() {
    state_->releaseAll();
    publishJoy();
  });

  const double rate_hz = this->get_parameter("publish_rate_hz").as_double();
  timer_ =
    this->create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), std::bind(&KeyboardJoy::timerCallback, this));

  std::printf(
    "\nkeyboard teleop: the gamepad's buttons on keys\n"
    "  w / s  hold: throttle (R2) / brake (L2)      a / d  hold: steer left / right\n"
    "  q / e  speed level up / down                 space  X: manual on/off\n"
    "  f      R1: dead man on/off (autonomy)        r      O: reverse on/off (w reverses slowly)\n"
    "  p      square: pit limit on/off (1 m/s)      Ctrl-C quit (stops the car)\n\n");
  if (!config.allow_autonomy) {
    std::printf("  (f is off here: allow_autonomy is false for this platform, so the keyboard can't arm autonomy)\n\n");
  }
  printStatus();
}

// The link check needs a terminal that answers the status query; without it, a dropped link would go
// unnoticed, so refuse to start
void KeyboardJoy::waitForTerminal(double timeout_s)
{
  const auto start = std::chrono::steady_clock::now();
  sendStatusQuery();
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < timeout_s) {
    pollfd fd{STDIN_FILENO, POLLIN, 0};
    if (poll(&fd, 1, 50) > 0 && (fd.revents & POLLIN)) {
      char buf[64];
      const ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
      for (ssize_t i = 0; i < n; ++i) {
        char key;
        if (input_.feed(buf[i], key) == TerminalEvent::kStatusReply) {
          link_->onReply(this->now().seconds());
          return;
        }
      }
    }
  }
  tcsetattr(STDIN_FILENO, TCSANOW, &*saved_termios_);
  saved_termios_.reset();
  throw std::runtime_error(
    "the terminal did not answer the status query (ESC [ 5 n), so a dropped link could not be detected. "
    "Use a standard terminal (GNOME Terminal, VS Code, kitty, ...)");
}

void KeyboardJoy::sendStatusQuery()
{
  std::fputs(TerminalInput::kStatusQuery, stdout);
  std::fflush(stdout);
}

KeyboardJoy::~KeyboardJoy()
{
  if (saved_termios_) {
    tcsetattr(STDIN_FILENO, TCSANOW, &*saved_termios_);
  }
  std::printf("\n");
}

void KeyboardJoy::timerCallback()
{
  const double now_s = this->now().seconds();
  if (!readKeys(now_s)) {
    rclcpp::shutdown();
    return;
  }
  if (link_->pingDue(now_s)) {
    sendStatusQuery();
  }
  if (link_->lost(now_s) && !link_lost_) {
    link_lost_ = true;
    state_->releaseAll();
  } else if (!link_->lost(now_s) && link_lost_) {
    link_lost_ = false;
    note_ = "link back: everything was released, press space / f again";
    note_until_s_ = now_s + 5.0;
  }
  state_->update(now_s);
  publishJoy();
  printStatus();
}

// false: quit (Ctrl-C, or the terminal hung up)
bool KeyboardJoy::readKeys(double now_s)
{
  pollfd fd{STDIN_FILENO, POLLIN, 0};
  if (poll(&fd, 1, 0) < 0 || (fd.revents & (POLLHUP | POLLERR | POLLNVAL))) {
    return false;
  }
  if (!(fd.revents & POLLIN)) {
    return true;
  }
  char buf[64];
  const ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
  if (n == 0) {
    return false;  // readable but empty: end of input
  }
  for (ssize_t i = 0; i < n; ++i) {
    // arrow and function keys arrive as escape sequences, which TerminalInput drops
    char key;
    switch (input_.feed(buf[i], key)) {
      case TerminalEvent::kStatusReply:
        link_->onReply(now_s);
        break;
      case TerminalEvent::kKey:
        switch (state_->handleKey(key, now_s)) {
          case KeyEvent::kQuit:
            return false;
          case KeyEvent::kRefused:
            note_ = "f: the keyboard can't arm autonomy here (allow_autonomy: false)";
            note_until_s_ = now_s + 3.0;
            break;
          case KeyEvent::kChanged:
          case KeyEvent::kIgnored:
            break;
        }
        break;
      case TerminalEvent::kNone:
        break;
    }
  }
  return true;
}

void KeyboardJoy::publishJoy()
{
  sensor_msgs::msg::Joy msg;
  msg.header.frame_id = my_frame_id;
  msg.header.stamp = this->now();
  msg.axes = state_->axes();
  msg.buttons = state_->buttons();
  joy_pub_->publish(msg);
}

void KeyboardJoy::printStatus()
{
  char line[256];
  const bool show_note = !note_.empty() && this->now().seconds() < note_until_s_;
  std::snprintf(
    line,
    sizeof(line),
    "  %-10s  level %.1f m/s  %s %s  steer %+5.2f  X %-3s  R1 %-3s  O %-3s  sq %-3s%s%s",
    link_lost_ ? "LINK LOST" : mode_.c_str(),
    state_->speedLevel(),
    state_->throttleHeld() ? "[w]" : " w ",
    state_->brakeHeld() ? "[s]" : " s ",
    state_->steering(),
    state_->safety() ? "on" : "off",
    !state_->autonomyAllowed() ? "n/a"
    : state_->autonomy()       ? "on"
                               : "off",
    state_->reverse() ? "on" : "off",
    state_->pitLimit() ? "on" : "off",
    show_note ? "  | " : "",
    show_note ? note_.c_str() : "");
  if (line != last_status_) {
    last_status_ = line;
    std::printf("\r%s\033[K", line);
    std::fflush(stdout);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<KeyboardJoy>();
    rclcpp::spin(node);
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "keyboard_joy_node: %s\n", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
