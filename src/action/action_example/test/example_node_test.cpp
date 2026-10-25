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

#include "action_example/example_node.hpp"

#include <memory>

#include <catch2/catch_test_macros.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "test_fixtures/test_executor_fixture.hpp"
#include "test_nodes/subscriber_test_node.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using wato::test::SubscriberTestNode;
using wato::test::TestExecutorFixture;

TEST_CASE_METHOD(TestExecutorFixture, "Example node publishes heartbeat when active", "[action_example]")
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"publish_rate_hz", 20.0}});

  auto node = std::make_shared<action_example::ExampleNode>(options);
  add_node(node);

  auto sub = std::make_shared<SubscriberTestNode<std_msgs::msg::String>>("/heartbeat", "heartbeat_sub");
  add_node(sub);

  start_spinning();

  REQUIRE(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_INACTIVE);
  REQUIRE(node->trigger_transition(Transition::TRANSITION_ACTIVATE).id() == State::PRIMARY_STATE_ACTIVE);
  REQUIRE(sub->wait_for_publishers(1));

  auto future = sub->expect_next_message();
  REQUIRE(future.get().data.rfind("action alive", 0) == 0);
}

TEST_CASE_METHOD(TestExecutorFixture, "Example node rejects invalid rate", "[action_example]")
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"publish_rate_hz", 0.0}});

  auto node = std::make_shared<action_example::ExampleNode>(options);
  add_node(node);

  REQUIRE(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_UNCONFIGURED);
}
