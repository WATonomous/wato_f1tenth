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

#ifndef TERMINAL_LINK
#define TERMINAL_LINK

#include <string>

// The terminal side of keyboard_joy_node, with no ROS in it so it can be unit tested.
//
// TerminalInput splits the bytes from the terminal into keys and escape sequences. Arrow and function
// keys arrive as sequences (ESC [ A, ESC O P, ...) and are dropped, so their letters never act as
// keys. The one sequence it reports is the answer to the device status query, ESC [ 0 n.
//
// LinkMonitor checks that the terminal at the other end is still there. Over SSH, a network drop
// can leave the terminal open on the car for hours until the SSH server notices, so the node asks
// the terminal for its status (ESC [ 5 n) every ping_period_s. Every common terminal answers that
// by itself, without showing anything. No answer for timeout_s means the link is lost.

enum class TerminalEvent
{
  kNone,  // in the middle of a sequence, or a sequence we don't use
  kKey,
  kStatusReply,  // ESC [ 0 n: the terminal answered the status query
};

class TerminalInput
{
public:
  // One byte from the terminal; for kKey, `key` is set
  TerminalEvent feed(char byte, char & key)
  {
    switch (state_) {
      case State::kText:
        if (byte == kEsc) {
          state_ = State::kEscape;
          return TerminalEvent::kNone;
        }
        key = byte;
        return TerminalEvent::kKey;
      case State::kEscape:
        if (byte == '[') {
          state_ = State::kCsi;
          params_.clear();
        } else if (byte == 'O') {
          state_ = State::kSs3;  // ESC O P (F1) and application-mode arrows: one more byte
        } else {
          state_ = State::kText;  // ESC + key (Alt+key): drop both
        }
        return TerminalEvent::kNone;
      case State::kSs3:
        state_ = State::kText;
        return TerminalEvent::kNone;
      case State::kCsi:
        // parameter (0x30-0x3F) and intermediate (0x20-0x2F) bytes, then one final byte (0x40-0x7E)
        if (byte >= 0x20 && byte <= 0x3F) {
          if (params_.size() < kMaxParams) {
            params_ += byte;
          }
          return TerminalEvent::kNone;
        }
        state_ = State::kText;
        if (byte == 'n' && params_ == "0") {
          return TerminalEvent::kStatusReply;
        }
        return TerminalEvent::kNone;
    }
    return TerminalEvent::kNone;
  }

  static constexpr const char * kStatusQuery = "\033[5n";

private:
  static constexpr char kEsc = 0x1b;
  static constexpr std::size_t kMaxParams = 16;

  enum class State
  {
    kText,
    kEscape,
    kCsi,
    kSs3,
  };

  State state_ = State::kText;
  std::string params_;
};

class LinkMonitor
{
public:
  LinkMonitor(double ping_period_s, double timeout_s)
  : ping_period_s_(ping_period_s)
  , timeout_s_(timeout_s)
  {}

  // true: send the status query now
  bool pingDue(double now_s)
  {
    if (last_ping_s_ >= 0.0 && now_s - last_ping_s_ < ping_period_s_) {
      return false;
    }
    last_ping_s_ = now_s;
    return true;
  }

  void onReply(double now_s)
  {
    last_reply_s_ = now_s;
  }

  // The terminal has answered at least once
  bool up() const
  {
    return last_reply_s_ >= 0.0;
  }

  // Answered before, but not for timeout_s
  bool lost(double now_s) const
  {
    return up() && now_s - last_reply_s_ > timeout_s_;
  }

private:
  double ping_period_s_, timeout_s_;
  double last_ping_s_ = -1.0;
  double last_reply_s_ = -1.0;
};

#endif
