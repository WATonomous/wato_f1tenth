#ifndef LOCAL_PLANNING_ROS_PLANNER_DIAGNOSTICS_HPP
#define LOCAL_PLANNING_ROS_PLANNER_DIAGNOSTICS_HPP

#include "local_planning/planning/local_planner.hpp"

#include <rclcpp/rclcpp.hpp>

#include <array>
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
  // Empty reports every intent. Otherwise only the listed intents are profiled,
  // so the usual "everything but the steady lap" case is a list, not a choice
  // of one.
  std::vector<PlannerIntent> profiling_intent_filter;
  double vehicle_full_width_m = 0.0;
  double compat_heading_rad = 0.0;
};

struct RosBoundaryProfile
{
  double cycle_ms = 0.0;
  double odom_conversion_ms = 0.0;
  double state_update_ms = 0.0;
  double planner_ms = 0.0;
  double decision_publish_ms = 0.0;
  double path_message_ms = 0.0;
  double tf_ms = 0.0;
  double path_publish_ms = 0.0;
  double marker_publish_ms = 0.0;
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
  LocalPlanProfile core;
  RosBoundaryProfile ros;
  CycleOutcome outcome;
};

class PlannerDiagnostics
{
public:
  PlannerDiagnostics(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    PlannerDiagnosticsConfig config);

  void recordGridUpdate(double update_ms, const OccupancyGrid & grid);
  void recordCycle(CycleProfile sample);

private:
  void recordProfile(CycleProfile sample);
  void emitProfile(PlannerIntent intent, std::vector<CycleProfile> & window);
  void noteTransitions(CycleProfile & sample);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  PlannerDiagnosticsConfig config_;
  std::array<std::vector<CycleProfile>, 4> profiling_windows_;
  std::vector<double> grid_profiling_window_;
  std::size_t grid_updates_since_report_ = 0;
  int grid_width_ = 0;
  int grid_height_ = 0;
  double grid_resolution_ = 0.0;
  bool has_previous_cycle_ = false;
  PlannerIntent previous_intent_ = PlannerIntent::FOLLOW_RACING_LINE;
  bool previous_path_published_ = false;
  double previous_terminal_d_m_ = 0.0;
  bool previous_steering_fresh_ = false;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_ROS_PLANNER_DIAGNOSTICS_HPP
