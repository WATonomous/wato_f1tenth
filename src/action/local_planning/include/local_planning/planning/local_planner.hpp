#ifndef LOCAL_PLANNING_PLANNING_LOCAL_PLANNER_HPP
#define LOCAL_PLANNING_PLANNING_LOCAL_PLANNER_HPP

#include "local_planning/collision/collision_checker.hpp"
#include "local_planning/maneuvers/maneuver_builder.hpp"
#include "local_planning/selection/candidate_selector.hpp"
#include "local_planning/state/racing_state_machine.hpp"

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

struct PlannerDecisionData
{
  PlannerIntent requested_intent = PlannerIntent::FOLLOW_RACING_LINE;
  RelativePosition relative_position = RelativePosition::NONE;
  bool opponent_detected = false;
  double opponent_gap_m = 0.0;
  ExecutedMode executed_mode = ExecutedMode::NO_LOCAL_PATH;
  CandidateSource candidate_source = CandidateSource::NONE;
  bool projection_seed_was_stale = false;
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
  uint32_t valid_candidate_count = 0;
  double cycle_time_ms = 0.0;
};

struct LocalPlanResult
{
  std::vector<ManeuverCandidate> pool;
  std::vector<EvaluatedCandidate> evaluated;
  int selected_index = -1;
  PlannerDecisionData decision;
};

class LocalPlanner
{
public:
  LocalPlanner(
    const RacelineReference & reference,
    const ManeuverBuilder & builder,
    LocalPlannerConfig config);

  LocalPlanner(const LocalPlanner &) = delete;
  LocalPlanner(LocalPlanner &&) = delete;
  LocalPlanner & operator=(const LocalPlanner &) = delete;
  LocalPlanner & operator=(LocalPlanner &&) = delete;

  void buildGridCache(OccupancyGrid & grid) const;
  LocalPlanResult plan(
    const TacticalState & state,
    const BoundaryState & ego,
    const OccupancyGrid & grid) const;

private:
  const RacelineReference & reference_;
  const ManeuverBuilder & builder_;
  LocalPlannerConfig config_;
  CollisionChecker collision_checker_;
  CandidateSelector selector_;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_PLANNING_LOCAL_PLANNER_HPP
