#ifndef LOCAL_PLANNING_STATE_RACING_STATE_MACHINE_HPP
#define LOCAL_PLANNING_STATE_RACING_STATE_MACHINE_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/reference/raceline_reference.hpp"

#include <cstdint>

namespace local_planning
{

struct StateMachineConfig
{
  double corridor_half_width_m = 0.25;

  double pass_enter_gap_m = 0.65;
  double pass_exit_gap_m = 0.95;
  double merge_enter_gap_m = -1.20;
  double merge_exit_gap_m = -0.80;
  double engagement_enter_gap_m = 2.00;
  double engagement_exit_gap_m = 2.50;
  double follow_enter_abs_d_m = 0.14;
  double follow_exit_abs_d_m = 0.28;
  double fast_confirmation_s = 0.05;
  double slow_confirmation_s = 0.15;
  uint32_t opponent_confirmation_grids = 2;
  uint32_t pass_merge_confirmation_grids = 3;
  uint32_t merge_pass_confirmation_grids = 3;
  uint32_t merge_probe_confirmation_cycles = 3;

  // A wrong-way backstop, not a tracking tolerance: lateral offset alone decides
  // the handoff. See local_planner.yaml for why this sits at 60 deg.
  double compat_heading_rad = 1.05;
};

enum class TransitionClass : uint8_t
{
  NONE = 0,
  FAST = 1,
  SLOW = 2
};

enum class TransitionReason : uint8_t
{
  NONE = 0,
  OPPONENT_ENGAGED = 1,
  PASS_BAND_ENTERED = 2,
  OPPONENT_CLEARED = 3,
  OPPONENT_CLEARANCE_LOST = 4,
  RACELINE_COMPATIBLE = 5,
  RACELINE_DEPARTED = 6,
  OPPONENT_LOST = 7,
  STATE_CORRECTION = 8
};

struct StateUpdateContext
{
  double now_s = 0.0;
  uint64_t costmap_sequence = 0;
  double costmap_stamp_s = 0.0;
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
  PlannerIntent proposed_intent = PlannerIntent::FOLLOW_RACING_LINE;
  TransitionClass transition_class = TransitionClass::NONE;
  TransitionReason transition_reason = TransitionReason::NONE;
  TransitionReason committed_transition_reason = TransitionReason::NONE;
  double pending_duration_s = 0.0;
  uint32_t pending_grid_count = 0;
  uint32_t merge_probe_valid_cycles = 0;
  bool merge_probe_available = false;
  uint64_t costmap_sequence = 0;
  double costmap_stamp_s = 0.0;
  uint64_t opponent_observation_sequence = 0;
  RelativePosition relative_position = RelativePosition::NONE;
  OpponentObservation opponent;

  // Outputs, not telemetry: the planner reads these instead of projecting again.
  double ego_s = 0.0;
  double ego_d = 0.0;

  double heading_error_rad = 0.0;
  bool raceline_compatible = false;

  // The seed window held no plausible foot and the whole loop had to be
  // searched.  Steady laps never set this, so a run of them means the seed has
  // stopped tracking ego and ego_s -- with every threshold that reads it -- is
  // not to be trusted.
  bool ego_seed_was_stale = false;

  // The projection stayed in the seed window but needed a wider tangent
  // tolerance than configured to find a foot there.  Distinct from the flag
  // above and much weaker: ego_s is still trustworthy, ego was just yawed away
  // from the reference.  Expected mid-slide; persistent outside a slide means
  // tangent_tolerance_rad is tighter than the car actually drives.
  bool ego_heading_check_relaxed = false;
};

// Stateful tactical layer. Safety remains in LocalPlanner; this class only
// debounces tactical intent.
class RacingStateMachine
{
public:
  // reference is borrowed and must outlive this object. Never pass a temporary.
  RacingStateMachine(
    const RacelineReference & reference,
    StateMachineConfig config,
    VehicleGeometry vehicle_geometry,
    GridPolicy grid_policy);

  RacingStateMachine(RacingStateMachine &&) = delete;
  RacingStateMachine(const RacingStateMachine &) = delete;
  RacingStateMachine & operator=(const RacingStateMachine &) = delete;
  RacingStateMachine & operator=(RacingStateMachine &&) = delete;

  // Must be called before state(): the ego projection is computed here.
  void update(
    const Odometry & ego_odom,
    const OccupancyGrid & occupancy_grid,
    StateUpdateContext context);
  // Convenience for callers that do not own timestamps/sequences. Each call is
  // treated as one new observation at the configured fast interval.
  void update(const Odometry & ego_odom, const OccupancyGrid & occupancy_grid);
  void reportMergeProbe(bool available);
  void resetEvidence();

  const TacticalState & state() const {return state_;}
  const StateMachineConfig & config() const {return config_;}

private:
  bool detectOpponent(
    const OccupancyGrid & occupancy_grid,
    double ego_s,
    OpponentObservation & out) const;

  bool corridorOccupied(const OccupancyGrid & occupancy_grid, double s) const;

  RelativePosition classify(double gap_m) const;

  // Records heading_error_rad and raceline_compatible on state_ as a side
  // effect, so the decision and the numbers behind it cannot drift apart.
  bool isRacelineCompatible(const Odometry & ego_odom, double ego_s, double ego_d);

  PlannerIntent proposedIntent(const Odometry & ego_odom) const;
  TransitionClass transitionClass(PlannerIntent proposed) const;
  TransitionReason transitionReason(PlannerIntent proposed) const;
  bool proposalUsesOpponent(PlannerIntent proposed) const;
  bool confirmationSatisfied(PlannerIntent proposed) const;

  const RacelineReference & reference_;
  StateMachineConfig config_;
  VehicleGeometry vehicle_geometry_;
  GridPolicy grid_policy_;
  TacticalState state_;
  OpponentObservation observed_opponent_;
  uint64_t last_costmap_sequence_ = 0;
  uint64_t automatic_sequence_ = 0;
  double last_update_s_ = 0.0;
  double pending_since_s_ = 0.0;
  uint64_t pending_last_grid_sequence_ = 0;
  bool has_time_ = false;
  bool opponent_engaged_ = false;

  // Carried between cycles. Stale-seed recovery inside RacelineReference handles
  // startup and relocalization.
  double ego_seed_s_ = 0.0;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_STATE_RACING_STATE_MACHINE_HPP
