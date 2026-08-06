#ifndef LOCAL_PLANNING_STATE_RACING_STATE_MACHINE_HPP
#define LOCAL_PLANNING_STATE_RACING_STATE_MACHINE_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/reference/raceline_reference.hpp"

namespace local_planning
{

struct StateMachineConfig
{
  double corridor_half_width_m = 0.25;

  double overlap_gap_m = 0.80;          // BEHIND <-> OVERLAPPING
  double clear_gap_m = 1.50;            // AHEAD_NOT_CLEAR <-> AHEAD_AND_CLEAR
  double overtake_start_gap_m = 3.00;   // must stay below the planner's horizon_m

  double compat_lateral_m = 0.40;       // keep equal to ManeuverConfig::sideDeadbandM()
  double compat_heading_rad = 0.15;
};

struct OpponentObservation
{
  bool detected = false;
  // Nearest observed station: the visible near face, and the anchor
  // ManeuverBuilder::overtake wants.
  double s = 0.0;
  double gap_m = 0.0;   // deltaS(ego_s, s); positive when the opponent is ahead
};

struct TacticalState
{
  PlannerIntent intent = PlannerIntent::FOLLOW_RACING_LINE;
  RelativePosition relative_position = RelativePosition::NONE;
  OpponentObservation opponent;

  // Outputs, not telemetry: the planner reads these instead of projecting again.
  double ego_s = 0.0;
  double ego_d = 0.0;
};

// Tactical layer: observation to intent, with no memory beyond the projection
// seed.
class RacingStateMachine
{
public:
  // reference is borrowed and must outlive this object. Never pass a temporary.
  RacingStateMachine(const RacelineReference & reference, StateMachineConfig config);

  RacingStateMachine(RacingStateMachine &&) = delete;
  RacingStateMachine(const RacingStateMachine &) = delete;
  RacingStateMachine & operator=(const RacingStateMachine &) = delete;
  RacingStateMachine & operator=(RacingStateMachine &&) = delete;

  // Must be called before state(): the ego projection is computed here.
  void update(const Odometry & ego_odom, const OccupancyGrid & occupancy_grid);

  const TacticalState & state() const {return state_;}
  const StateMachineConfig & config() const {return config_;}

private:
  bool detectOpponent(
    const OccupancyGrid & occupancy_grid,
    double ego_s,
    OpponentObservation & out) const;

  bool corridorOccupied(const OccupancyGrid & occupancy_grid, double s) const;

  RelativePosition classify(double gap_m) const;

  bool isRacelineCompatible(const Odometry & ego_odom, double ego_s, double ego_d) const;

  PlannerIntent nextIntent(const Odometry & ego_odom) const;

  const RacelineReference & reference_;
  StateMachineConfig config_;
  TacticalState state_;

  // Carried between cycles. Stale-seed recovery inside RacelineReference handles
  // startup and relocalization.
  double ego_seed_s_ = 0.0;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_STATE_RACING_STATE_MACHINE_HPP
