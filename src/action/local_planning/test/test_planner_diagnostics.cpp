#include "local_planning/ros/planner_diagnostics.hpp"

#include <gtest/gtest.h>
#include <rcutils/logging.h>

#include <array>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace local_planning
{
namespace
{

class LogCapture
{
public:
  LogCapture()
  : previous_(rcutils_logging_get_output_handler())
  {
    active_ = this;
    rcutils_logging_set_output_handler(&LogCapture::handle);
  }

  ~LogCapture()
  {
    rcutils_logging_set_output_handler(previous_);
    active_ = nullptr;
  }

  std::size_t count(const std::string & token) const
  {
    std::size_t result = 0;
    for (const auto & message : messages_) {
      result += message.find(token) != std::string::npos ? 1U : 0U;
    }
    return result;
  }

  const std::vector<std::string> & messages() const {return messages_;}

private:
  static void handle(
    const rcutils_log_location_t *, int, const char *, rcutils_time_point_value_t,
    const char * format, va_list * args)
  {
    if (active_ == nullptr) {
      return;
    }
    std::array<char, 8192> buffer{};
    va_list copied;
    va_copy(copied, *args);
    std::vsnprintf(buffer.data(), buffer.size(), format, copied);
    va_end(copied);
    active_->messages_.emplace_back(buffer.data());
  }

  static LogCapture * active_;
  rcutils_logging_output_handler_t previous_;
  std::vector<std::string> messages_;
};

LogCapture * LogCapture::active_ = nullptr;

PlannerDiagnostics makeDiagnostics(PlannerDiagnosticsConfig config)
{
  return PlannerDiagnostics(
    rclcpp::get_logger("planner_diagnostics_test"),
    std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME),
    std::move(config));
}

TEST(PlannerDiagnostics, AggregatesPerIntentPercentilesAndGridSamples)
{
  PlannerDiagnosticsConfig config;
  config.profiling_log_every_n_cycles = 3;
  auto diagnostics = makeDiagnostics(config);
  OccupancyGrid grid;
  grid.width = 4;
  grid.height = 3;
  grid.resolution = 0.25;
  diagnostics.recordGridUpdate(1.0, grid);
  diagnostics.recordGridUpdate(9.0, grid);
  diagnostics.recordGridUpdate(5.0, grid);
  LogCapture capture;

  for (const double cycle_ms : {1.0, 2.0, 9.0}) {
    PlannerCycleProfile sample;
    sample.cycle_ms = cycle_ms;
    sample.inputs_ready = true;
    diagnostics.recordCycle(sample, nullptr, false, false);
  }

  ASSERT_EQ(capture.count("LOCAL_PLANNER_PROFILE"), 1U);
  const std::string & message = capture.messages().back();
  EXPECT_NE(message.find("intent=FOLLOW_RACING_LINE"), std::string::npos);
  EXPECT_NE(message.find("window=3 ready=3"), std::string::npos);
  EXPECT_NE(message.find("cycle_ms=4.000/9.000/9.000"), std::string::npos);
  EXPECT_NE(message.find("grid_updates=3 grid=4x3 cells=12 res=0.2500"), std::string::npos);
  EXPECT_NE(message.find("grid_ms=5.000/9.000/9.000"), std::string::npos);
}

TEST(PlannerDiagnostics, IntentFilterIgnoresOtherWindows)
{
  PlannerDiagnosticsConfig config;
  config.profiling_log_every_n_cycles = 1;
  config.profiling_intent_filter = PlannerIntent::OVERTAKE;
  auto diagnostics = makeDiagnostics(config);
  LogCapture capture;

  PlannerCycleProfile follow;
  diagnostics.recordCycle(follow, nullptr, false, false);
  EXPECT_EQ(capture.count("LOCAL_PLANNER_PROFILE"), 0U);

  PlannerCycleProfile overtake;
  overtake.intent = PlannerIntent::OVERTAKE;
  diagnostics.recordCycle(overtake, nullptr, false, false);
  ASSERT_EQ(capture.count("LOCAL_PLANNER_PROFILE"), 1U);
  EXPECT_NE(capture.messages().back().find("intent=OVERTAKE"), std::string::npos);
}

TEST(PlannerDiagnostics, LogsOnlyEdgeTriggeredControllerVisibleTransitions)
{
  PlannerDiagnosticsConfig config;
  config.profiling_enabled = false;
  config.vehicle_full_width_m = 0.4;
  config.compat_heading_rad = 1.05;
  auto diagnostics = makeDiagnostics(config);
  LogCapture capture;

  PlannerDecisionData decision;
  decision.terminal_d_m = 0.3;
  diagnostics.recordCycle({}, &decision, false, false);
  EXPECT_EQ(capture.count("LOCAL_PLANNER_EVENT"), 0U);

  decision.requested_intent = PlannerIntent::OVERTAKE;
  diagnostics.recordCycle({}, &decision, false, false);
  decision.requested_intent = PlannerIntent::OVERTAKE;
  diagnostics.recordCycle({}, &decision, true, false);
  diagnostics.recordCycle({}, &decision, true, true);
  decision.terminal_d_m = -0.2;
  diagnostics.recordCycle({}, &decision, true, true);
  diagnostics.recordCycle({}, &decision, true, true);

  EXPECT_EQ(capture.count("LOCAL_PLANNER_EVENT"), 4U);
  EXPECT_NE(capture.messages().front().find("FOLLOW_RACING_LINE->OVERTAKE"), std::string::npos);
  EXPECT_NE(capture.messages()[1].find("path=no->yes"), std::string::npos);
  EXPECT_NE(capture.messages()[2].find("steer_fresh=0->1"), std::string::npos);
  EXPECT_NE(capture.messages()[3].find("term_d=+0.300->-0.200"), std::string::npos);
}

}  // namespace
}  // namespace local_planning
