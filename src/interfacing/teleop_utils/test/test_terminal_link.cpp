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

#include <string>
#include <vector>

#include <catch2/catch_all.hpp>

#include "terminal_link.hpp"  // NOLINT(build/include_subdir)

namespace
{
struct Fed
{
  std::string keys;
  int replies = 0;
};

Fed feedAll(TerminalInput & input, const std::string & bytes)
{
  Fed fed;
  for (char byte : bytes) {
    char key = 0;
    switch (input.feed(byte, key)) {
      case TerminalEvent::kKey:
        fed.keys += key;
        break;
      case TerminalEvent::kStatusReply:
        ++fed.replies;
        break;
      case TerminalEvent::kNone:
        break;
    }
  }
  return fed;
}
}  // namespace

TEST_CASE("plain keys pass through, including Ctrl-C", "[terminal_link]")
{
  TerminalInput input;
  const auto fed = feedAll(input, std::string("w a\x03"));
  REQUIRE(fed.keys == std::string("w a\x03"));
  REQUIRE(fed.replies == 0);
}

TEST_CASE("the status reply is recognised, and not read as keys", "[terminal_link]")
{
  TerminalInput input;
  const auto fed = feedAll(input, "w\033[0nw");
  REQUIRE(fed.keys == "ww");
  REQUIRE(fed.replies == 1);
}

TEST_CASE("a reply split across reads is still one reply", "[terminal_link]")
{
  TerminalInput input;
  auto first = feedAll(input, "\033[");
  auto second = feedAll(input, "0n");
  REQUIRE(first.replies + second.replies == 1);
  REQUIRE(first.keys.empty());
  REQUIRE(second.keys.empty());
}

TEST_CASE("arrow, function and Alt keys never act as keys", "[terminal_link]")
{
  TerminalInput input;
  // up arrow (CSI A), F1 (SS3 P), application-mode right arrow (SS3 C), Delete (CSI 3 ~),
  // Alt+w (ESC w), and a status-like sequence with other parameters
  const auto fed = feedAll(input, "\033[A\033OP\033OC\033[3~\033w\033[5n\033[1;5D");
  REQUIRE(fed.keys.empty());
  REQUIRE(fed.replies == 0);
  // and keys after them still work
  REQUIRE(feedAll(input, "d").keys == "d");
}

TEST_CASE("the link counts as lost only after it was up and then went quiet", "[terminal_link]")
{
  LinkMonitor link(0.25, 1.0);
  REQUIRE_FALSE(link.up());
  REQUIRE_FALSE(link.lost(100.0));  // never answered: the node refuses to start instead

  link.onReply(0.0);
  REQUIRE(link.up());
  REQUIRE_FALSE(link.lost(0.9));
  REQUIRE(link.lost(1.1));

  link.onReply(1.2);  // answers again: back up
  REQUIRE_FALSE(link.lost(1.3));
}

TEST_CASE("pings go out every ping period", "[terminal_link]")
{
  LinkMonitor link(0.25, 1.0);
  std::vector<double> sent;
  for (double t = 0.0; t < 1.0; t += 0.05) {
    if (link.pingDue(t)) {
      sent.push_back(t);
    }
  }
  REQUIRE(sent.size() == 4);
  REQUIRE(sent.front() == 0.0);
}
