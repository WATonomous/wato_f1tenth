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

#include <catch2/catch_all.hpp>

#include "keyboard_teleop_state.hpp"  // NOLINT(build/include_subdir)

namespace
{
KeyboardTeleopState makeState()
{
  return KeyboardTeleopState(KeyboardTeleopLimits{1.0, 0.5, 0.25, 0.5, 0.1});
}
}  // namespace

TEST_CASE("space toggles the dead man", "[keyboard_teleop]")
{
  auto state = makeState();
  REQUIRE_FALSE(state.deadMan());
  REQUIRE(state.handleKey(' ') == KeyEvent::kDeadMan);
  REQUIRE(state.deadMan());
  REQUIRE(state.handleKey(' ') == KeyEvent::kDeadMan);
  REQUIRE_FALSE(state.deadMan());
}

TEST_CASE("manual takes over and releases the dead man", "[keyboard_teleop]")
{
  auto state = makeState();
  state.handleKey(' ');
  REQUIRE(state.handleKey('m') == KeyEvent::kManualOn);
  REQUIRE(state.manual());
  REQUIRE_FALSE(state.deadMan());

  SECTION("space is ignored while manual")
  {
    REQUIRE(state.handleKey(' ') == KeyEvent::kIgnored);
    REQUIRE_FALSE(state.deadMan());
  }

  SECTION("leaving manual stops and does not re-arm autonomy")
  {
    state.handleKey('w');
    REQUIRE(state.handleKey('M') == KeyEvent::kManualOff);
    REQUIRE_FALSE(state.manual());
    REQUIRE_FALSE(state.deadMan());
    REQUIRE(state.speed() == 0.0);
  }
}

TEST_CASE("drive keys only work in manual, within the limits", "[keyboard_teleop]")
{
  auto state = makeState();
  REQUIRE(state.handleKey('w') == KeyEvent::kIgnored);
  REQUIRE(state.speed() == 0.0);

  state.handleKey('m');
  for (int i = 0; i < 10; ++i) {
    state.handleKey('w');
    state.handleKey('a');
  }
  REQUIRE(state.speed() == Catch::Approx(1.0));
  REQUIRE(state.steering() == Catch::Approx(0.5));

  for (int i = 0; i < 20; ++i) {
    state.handleKey('s');
    state.handleKey('d');
  }
  REQUIRE(state.speed() == Catch::Approx(-0.5));
  REQUIRE(state.steering() == Catch::Approx(-0.5));

  REQUIRE(state.handleKey('x') == KeyEvent::kDrive);
  REQUIRE(state.speed() == 0.0);
  REQUIRE(state.steering() == 0.0);
}

TEST_CASE("quit keys and dead man sync", "[keyboard_teleop]")
{
  auto state = makeState();
  REQUIRE(state.handleKey('q') == KeyEvent::kQuit);
  REQUIRE(state.handleKey(0x03) == KeyEvent::kQuit);

  // the gamepad turned it on: the next space turns it off
  state.syncDeadMan(true);
  state.handleKey(' ');
  REQUIRE_FALSE(state.deadMan());
}
