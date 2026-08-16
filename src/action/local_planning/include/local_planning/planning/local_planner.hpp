#ifndef LOCAL_PLANNING_PLANNING_LOCAL_PLANNER_HPP
#define LOCAL_PLANNING_PLANNING_LOCAL_PLANNER_HPP

#include "local_planning/collision/collision_checker.hpp"
#include "local_planning/collision/track_bounds_checker.hpp"
#include "local_planning/maneuvers/maneuver_builder.hpp"
#include "local_planning/selection/candidate_selector.hpp"
#include "local_planning/speed/velocity_profile.hpp"
#include "local_planning/state/racing_state_machine.hpp"

#include <cstdint>
#include <vector>

namespace local_planning
{

enum class ExecutedMode : uint8_t
{
  NO_LOCAL_PATH = 0,
  MANEUVER = 1,
  BRAKING_FALLBACK = 2,
  BRAKING_UNAVAILABLE = 3
};

enum class RecoveryReason : uint8_t
{
  NONE = 0,
  MERGE_PATH_UNAVAILABLE = 1,
  OPPONENT_CLEARANCE_LOST = 2,
  BRAKING_FALLBACK = 3,
  NO_SAFE_LOCAL_PATH = 4
};

struct PlannerDecisionData
{
  PlannerIntent requested_intent = PlannerIntent::FOLLOW_RACING_LINE;
  PlannerIntent proposed_intent = PlannerIntent::FOLLOW_RACING_LINE;
  PlannerIntent executed_intent = PlannerIntent::FOLLOW_RACING_LINE;
  TransitionClass transition_class = TransitionClass::NONE;
  TransitionReason transition_reason = TransitionReason::NONE;
  RecoveryReason recovery_reason = RecoveryReason::NONE;
  double pending_transition_s = 0.0;
  uint32_t pending_grid_count = 0;
  uint32_t merge_probe_valid_cycles = 0;
  bool merge_probe_available = false;
  uint64_t costmap_sequence = 0;
  double costmap_stamp_s = 0.0;
  uint64_t opponent_observation_sequence = 0;
  RelativePosition relative_position = RelativePosition::NONE;
  bool opponent_detected = false;
  double opponent_gap_m = 0.0;
  // The two inputs to the FOLLOW/MERGE gate, plus its result.  Published so a
  // flapping intent can be read off a bag without rebuilding.
  // Where the gate thinks the car is.  Published alongside its own inputs
  // because a wrong station makes ego_d and heading_error_rad wrong together,
  // and that pair is indistinguishable from a genuine excursion without it.
  double ego_s_m = 0.0;
  double ego_d_m = 0.0;
  double heading_error_rad = 0.0;
  bool raceline_compatible = false;
  ExecutedMode executed_mode = ExecutedMode::NO_LOCAL_PATH;
  CandidateSource candidate_source = CandidateSource::NONE;
  bool selected_offset_tail = false;
  // The selected candidate's overshoot: worst |d| along its path.  On the wire
  // because it is what the port is judged on, and because it is the one part of
  // the executed geometry that terminal_d_m does not imply -- the connection
  // starts from the measured d' and d'', which can carry it wide of the offset
  // it was commanded to reach.
  double selected_max_abs_d_m = 0.0;
  bool projection_seed_was_stale = false;
  bool projection_heading_check_relaxed = false;
  CollisionStatus clearance_class = CollisionStatus::OUT_OF_GRID;
  double minimum_clearance_m = 0.0;
  double max_abs_curvature_inv_m = 0.0;
  double min_speed_mps = 0.0;
  double max_speed_mps = 0.0;
  double start_curvature_inv_m = 0.0;
  bool start_curvature_from_steering = false;
  double terminal_d_m = 0.0;
  double best_cost_s = 0.0;
  double median_cost_s = 0.0;
  uint32_t generated_count = 0;
  uint32_t collision_rejected = 0;
  uint32_t out_of_grid_rejected = 0;
  uint32_t velocity_rejected = 0;
  bool track_bounds_ready = false;
  double sustainable_left_m = 0.0;
  double sustainable_right_m = 0.0;
  uint32_t track_bounds_rejected = 0;
  uint32_t valid_candidate_count = 0;
  double cycle_time_ms = 0.0;
};

// Timings are intentionally kept out of PlannerDecisionData: they are
// diagnostics for stdout, not part of the planner's control contract.
struct LocalPlanProfile
{
  double candidate_generation_ms = 0.0;
  double collision_check_ms = 0.0;
  double track_bounds_ms = 0.0;
  double terminal_projection_ms = 0.0;
  double velocity_profile_ms = 0.0;
  double selection_ms = 0.0;
  double finalization_ms = 0.0;
  uint32_t total_path_samples = 0;
  uint32_t max_path_samples = 0;
  uint32_t collision_poses_checked = 0;
};

struct LocalPlanResult
{
  std::vector<ManeuverCandidate> pool;
  std::vector<EvaluatedCandidate> evaluated;
  int selected_index = -1;
  int merge_probe_index = -1;
  PlannerDecisionData decision;
  LocalPlanProfile profile;
};

class LocalPlanner
{
public:
  LocalPlanner(
    const RacelineReference & reference,
    const ManeuverBuilder & builder,
    VehicleGeometry vehicle_geometry,
    GridPolicy grid_policy,
    CollisionConfig collision_config,
    VelocityProfileConfig velocity_config);

  LocalPlanner(const LocalPlanner &) = delete;
  LocalPlanner(LocalPlanner &&) = delete;
  LocalPlanner & operator=(const LocalPlanner &) = delete;
  LocalPlanner & operator=(LocalPlanner &&) = delete;

  void buildGridCache(OccupancyGrid & grid) const;
  LocalPlanResult plan(
    const TacticalState & state,
    const BoundaryState & ego,
    const OccupancyGrid & grid) const;

  // Re-check an already-selected path against a newer grid.  Same checker and
  // config plan() ranks with, so a held path is judged by the same rule that
  // admitted it.
  CollisionCheckResult validatePath(const Path & path, const OccupancyGrid & grid) const
  {
    return collision_checker_.collisionCheck(path, grid);
  }

private:
  const RacelineReference & reference_;
  const ManeuverBuilder & builder_;
  VehicleGeometry vehicle_geometry_;
  VelocityProfileConfig velocity_config_;
  CollisionChecker collision_checker_;
  TrackBoundsChecker track_bounds_checker_;
  CandidateSelector selector_;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_PLANNING_LOCAL_PLANNER_HPP
