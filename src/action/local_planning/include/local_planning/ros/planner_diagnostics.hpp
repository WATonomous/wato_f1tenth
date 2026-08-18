#ifndef LOCAL_PLANNING_ROS_PLANNER_DIAGNOSTICS_HPP
#define LOCAL_PLANNING_ROS_PLANNER_DIAGNOSTICS_HPP

#include "local_planning/planning/local_planner.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace local_planning
{

struct PlannerDiagnosticsConfig
{
  bool profiling_enabled = true;
  int profiling_log_every_n_cycles = 20;
  bool diagnostics_enabled = true;
  double vehicle_full_width_m = 0.0;
  double compat_heading_rad = 0.0;
};

struct CycleOutcome
{
  bool inputs_ready = false;
  std::optional<PlannerDecisionData> decision;
  bool path_published = false;
  bool steering_fresh = false;
  bool intent_changed = false;
  bool side_flipped = false;
};

struct CycleProfile
{
  double cycle_ms = 0.0;
  CycleOutcome outcome;
};

class PlannerDiagnostics
{
public:
  PlannerDiagnostics(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    PlannerDiagnosticsConfig config);

  void recordCycle(CycleProfile sample);

private:
  void recordProfile(CycleProfile sample);
  void emitProfile();
  void noteTransitions(CycleProfile & sample);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  PlannerDiagnosticsConfig config_;
  std::vector<CycleProfile> profiling_window_;
  bool has_previous_cycle_ = false;
  PlannerIntent previous_intent_ = PlannerIntent::FOLLOW_RACING_LINE;
  PlannerIntent previous_proposed_intent_ = PlannerIntent::FOLLOW_RACING_LINE;
  uint32_t previous_pending_grid_count_ = 0;
  uint32_t previous_merge_probe_valid_cycles_ = 0;
  bool previous_path_published_ = false;
  double previous_terminal_d_m_ = 0.0;
  bool previous_steering_fresh_ = false;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_ROS_PLANNER_DIAGNOSTICS_HPP
