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
  std::optional<PlannerIntent> profiling_intent_filter;
  double vehicle_full_width_m = 0.0;
  double compat_heading_rad = 0.0;
};

struct PlannerCycleProfile
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
  double candidate_generation_ms = 0.0;
  double collision_check_ms = 0.0;
  double terminal_projection_ms = 0.0;
  double velocity_profile_ms = 0.0;
  double selection_ms = 0.0;
  double finalization_ms = 0.0;
  uint32_t candidate_count = 0;
  uint32_t total_path_samples = 0;
  uint32_t max_path_samples = 0;
  uint32_t collision_poses_checked = 0;
  uint32_t collision_rejected = 0;
  uint32_t out_of_grid_rejected = 0;
  uint32_t velocity_rejected = 0;
  uint32_t valid_candidate_count = 0;
  uint64_t station_hint_samples = 0;
  uint64_t station_hint_fallbacks = 0;
  bool path_published = false;
  bool steering_fresh = false;
  bool intent_changed = false;
  bool side_flipped = false;
  ExecutedMode executed_mode = ExecutedMode::NO_LOCAL_PATH;
  bool inputs_ready = false;
  PlannerIntent intent = PlannerIntent::FOLLOW_RACING_LINE;
};

class PlannerDiagnostics
{
public:
  PlannerDiagnostics(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    PlannerDiagnosticsConfig config);

  void recordGridUpdate(double update_ms, const OccupancyGrid & grid);
  void recordCycle(
    PlannerCycleProfile sample,
    const PlannerDecisionData * decision,
    bool path_published,
    bool steering_fresh);

private:
  void recordProfile(PlannerCycleProfile sample);
  void emitProfile(PlannerIntent intent, std::vector<PlannerCycleProfile> & window);
  void noteTransitions(
    const PlannerDecisionData & data,
    bool path_published,
    bool steering_fresh,
    PlannerCycleProfile & sample);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  PlannerDiagnosticsConfig config_;
  std::array<std::vector<PlannerCycleProfile>, 4> profiling_windows_;
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
