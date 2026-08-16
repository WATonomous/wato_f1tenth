#include "local_planning/state/racing_state_machine.hpp"

#include <cmath>
#include <cstddef>
#include <algorithm>
#include <utility>

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double wrapAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

bool gridIndex(const OccupancyGrid & grid, const Point & p, std::size_t & index)
{
  const int col = static_cast<int>(std::floor((p.x - grid.origin.x) / grid.resolution));
  const int row = static_cast<int>(std::floor((p.y - grid.origin.y) / grid.resolution));
  if (col < 0 || col >= grid.width || row < 0 || row >= grid.height) {
    return false;
  }
  index = static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.width) +
    static_cast<std::size_t>(col);
  return index < grid.data.size();
}

bool gridUsable(const OccupancyGrid & grid)
{
  return grid.resolution > 0.0 && grid.width > 0 && grid.height > 0 &&
         grid.data.size() >= static_cast<std::size_t>(grid.width) *
         static_cast<std::size_t>(grid.height);
}

} // namespace

RacingStateMachine::RacingStateMachine(
  const RacelineReference & reference,
  StateMachineConfig config,
  VehicleGeometry vehicle_geometry,
  GridPolicy grid_policy)
: reference_(reference), config_(std::move(config)),
  vehicle_geometry_(vehicle_geometry), grid_policy_(grid_policy)
{
}

void RacingStateMachine::update(
  const Odometry & ego_odom,
  const OccupancyGrid & occupancy_grid,
  StateUpdateContext context)
{
  if (!reference_.valid()) {
    state_ = TacticalState{};
    resetEvidence();
    return;
  }

  if (has_time_ && context.now_s < last_update_s_) {
    resetEvidence();
  }
  has_time_ = true;
  last_update_s_ = context.now_s;

  // Ego first: its s seeds the next cycle and bounds the opponent scan.
  const Projection ego = reference_.project(
    ego_odom.position, ego_odom.heading, ego_seed_s_);
  ego_seed_s_ = ego.s;
  state_.ego_s = ego.s;
  state_.ego_d = ego.d;
  state_.ego_seed_was_stale = ego.seed_was_stale;
  state_.ego_heading_check_relaxed = ego.heading_check_relaxed;

  const bool new_costmap = context.costmap_sequence != last_costmap_sequence_;
  state_.costmap_sequence = context.costmap_sequence;
  state_.costmap_stamp_s = context.costmap_stamp_s;
  if (new_costmap) {
    last_costmap_sequence_ = context.costmap_sequence;
    detectOpponent(occupancy_grid, ego.s, observed_opponent_);
    state_.opponent_observation_sequence = context.costmap_sequence;
  }
  state_.opponent = observed_opponent_;
  if (state_.opponent.detected) {
    // The occupied station belongs to the latest unique costmap. Gap is always
    // recomputed from live localization so repeated planner ticks do not reuse
    // an old ego position.
    state_.opponent.gap_m = reference_.deltaS(ego.s, state_.opponent.s);
    state_.relative_position = classify(state_.opponent.gap_m);
  } else {
    state_.relative_position = RelativePosition::NONE;
  }

  isRacelineCompatible(ego_odom, state_.ego_s, state_.ego_d);
  const PlannerIntent proposed = proposedIntent(ego_odom);
  if (proposed == state_.intent) {
    resetEvidence();
    state_.proposed_intent = state_.intent;
    return;
  }

  if (proposed != state_.proposed_intent) {
    state_.proposed_intent = proposed;
    state_.transition_class = transitionClass(proposed);
    state_.transition_reason = transitionReason(proposed);
    pending_since_s_ = context.now_s;
    pending_last_grid_sequence_ = 0;
    state_.pending_grid_count = 0;
  }
  state_.pending_duration_s = std::max(0.0, context.now_s - pending_since_s_);
  if (new_costmap && pending_last_grid_sequence_ != context.costmap_sequence) {
    pending_last_grid_sequence_ = context.costmap_sequence;
    ++state_.pending_grid_count;
  }

  if (confirmationSatisfied(proposed)) {
    state_.committed_transition_reason = state_.transition_reason;
    state_.intent = proposed;
    if (state_.intent == PlannerIntent::FOLLOW_RACING_LINE) {
      opponent_engaged_ = false;
    } else if (state_.opponent.detected) {
      opponent_engaged_ = true;
    }
    resetEvidence();
    state_.proposed_intent = state_.intent;
  }
}

void RacingStateMachine::update(
  const Odometry & ego_odom,
  const OccupancyGrid & occupancy_grid)
{
  ++automatic_sequence_;
  const double step = std::max(config_.fast_confirmation_s, 1e-3);
  const double automatic_time = automatic_sequence_ * step;
  update(ego_odom, occupancy_grid, StateUpdateContext{automatic_time,
      automatic_sequence_, automatic_time});
}

void RacingStateMachine::reportMergeProbe(bool available)
{
  state_.merge_probe_available = available;
  if (available) {
    ++state_.merge_probe_valid_cycles;
  } else {
    state_.merge_probe_valid_cycles = 0;
  }
}

void RacingStateMachine::resetEvidence()
{
  state_.proposed_intent = state_.intent;
  state_.transition_class = TransitionClass::NONE;
  state_.transition_reason = TransitionReason::NONE;
  state_.pending_duration_s = 0.0;
  state_.pending_grid_count = 0;
  state_.merge_probe_available = false;
  state_.merge_probe_valid_cycles = 0;
  pending_since_s_ = last_update_s_;
  pending_last_grid_sequence_ = 0;
}

// Nearest occupied station in the raceline corridor. See PRD 5 "As built" for why
// nearest-station is equivalent to grouping components and taking the near face.
bool RacingStateMachine::detectOpponent(
  const OccupancyGrid & occupancy_grid,
  double ego_s,
  OpponentObservation & out) const
{
  out = OpponentObservation{};
  if (!gridUsable(occupancy_grid)) {
    return false;
  }

  const double step = occupancy_grid.resolution;
  const double limit = 0.5 * reference_.totalLength();

  bool found = false;
  double nearest_offset_m = 0.0;

  // Forward first, so an equidistant tie resolves to the opponent ahead.
  for (const int direction : {1, -1}) {
    for (double offset = step; offset <= limit; offset += step) {
      if (found && offset >= std::abs(nearest_offset_m)) {
        break;
      }

      const double s = ego_s + direction * offset;

      // Leaving the grid ends this direction: nothing further out was observable.
      std::size_t unused = 0;
      if (!gridIndex(occupancy_grid, reference_.toCartesian(s, 0.0), unused)) {
        break;
      }

      if (corridorOccupied(occupancy_grid, s)) {
        found = true;
        nearest_offset_m = direction * offset;
        break;
      }
    }
  }

  if (!found) {
    return false;
  }

  out.detected = true;
  out.s = reference_.wrapS(ego_s + nearest_offset_m);
  out.gap_m = reference_.deltaS(ego_s, out.s);
  return true;
}

bool RacingStateMachine::corridorOccupied(
  const OccupancyGrid & occupancy_grid,
  double s) const
{
  const double half_width = config_.corridor_half_width_m;
  for (double d = -half_width; d <= half_width + 1e-9; d += occupancy_grid.resolution) {
    std::size_t index = 0;
    if (gridIndex(occupancy_grid, reference_.toCartesian(s, d), index) &&
      grid_policy_.isOccupied(occupancy_grid.data[index]))
    {
      return true;
    }
  }
  return false;
}

RelativePosition RacingStateMachine::classify(double gap_m) const
{
  if (gap_m >= config_.pass_exit_gap_m) {
    return RelativePosition::BEHIND;
  }
  if (gap_m > config_.merge_exit_gap_m) {
    return RelativePosition::OVERLAPPING;
  }
  if (gap_m > config_.merge_enter_gap_m) {
    return RelativePosition::AHEAD_NOT_CLEAR;
  }
  return RelativePosition::AHEAD_AND_CLEAR;
}

bool RacingStateMachine::isRacelineCompatible(
  const Odometry & ego_odom,
  double ego_s,
  double ego_d)
{
  // Computed before the lateral early-out so the telemetry is populated on
  // every cycle, not only the ones that reach the heading test.
  const ReferenceGeometrySample sample = reference_.sampleAtS(ego_s);
  state_.heading_error_rad = wrapAngle(ego_odom.heading - sample.heading);

  const double lateral_limit = state_.intent == PlannerIntent::FOLLOW_RACING_LINE ?
    config_.follow_exit_abs_d_m : config_.follow_enter_abs_d_m;
  state_.raceline_compatible =
    std::abs(ego_d) <= lateral_limit &&
    std::abs(state_.heading_error_rad) <= config_.compat_heading_rad;
  return state_.raceline_compatible;
}

PlannerIntent RacingStateMachine::proposedIntent(const Odometry & ego_odom) const
{
  (void)ego_odom;
  const bool compatible = state_.raceline_compatible;

  if (!state_.opponent.detected) {
    return compatible ? PlannerIntent::FOLLOW_RACING_LINE : PlannerIntent::MERGE;
  }

  const double gap = state_.opponent.gap_m;
  const double engagement_limit = opponent_engaged_ ?
    config_.engagement_exit_gap_m : config_.engagement_enter_gap_m;
  if (gap > engagement_limit) {
    return compatible ? PlannerIntent::FOLLOW_RACING_LINE : PlannerIntent::MERGE;
  }

  switch (state_.intent) {
    case PlannerIntent::FOLLOW_RACING_LINE:
      if (gap <= config_.merge_enter_gap_m) {
        return compatible ? PlannerIntent::FOLLOW_RACING_LINE : PlannerIntent::MERGE;
      }
      if (gap <= config_.pass_enter_gap_m) {return PlannerIntent::PASS;}
      if (!compatible) {return PlannerIntent::MERGE;}
      return PlannerIntent::OVERTAKE;
    case PlannerIntent::OVERTAKE:
      if (gap <= config_.merge_enter_gap_m) {
        return compatible ? PlannerIntent::FOLLOW_RACING_LINE : PlannerIntent::MERGE;
      }
      return gap <= config_.pass_enter_gap_m ? PlannerIntent::PASS : PlannerIntent::OVERTAKE;
    case PlannerIntent::PASS:
      if (gap >= config_.pass_exit_gap_m) {return PlannerIntent::OVERTAKE;}
      if (gap <= config_.merge_enter_gap_m) {return PlannerIntent::MERGE;}
      return PlannerIntent::PASS;
    case PlannerIntent::MERGE:
      if (gap >= config_.pass_exit_gap_m) {return PlannerIntent::OVERTAKE;}
      if (gap >= config_.merge_exit_gap_m) {return PlannerIntent::PASS;}
      return compatible ? PlannerIntent::FOLLOW_RACING_LINE : PlannerIntent::MERGE;
  }

  return PlannerIntent::MERGE;
}

TransitionClass RacingStateMachine::transitionClass(PlannerIntent proposed) const
{
  const bool normal_fast =
    (state_.intent == PlannerIntent::FOLLOW_RACING_LINE &&
    proposed == PlannerIntent::OVERTAKE) ||
    (state_.intent == PlannerIntent::OVERTAKE && proposed == PlannerIntent::PASS) ||
    (state_.intent == PlannerIntent::PASS && proposed == PlannerIntent::OVERTAKE) ||
    (state_.intent == PlannerIntent::OVERTAKE &&
    proposed == PlannerIntent::FOLLOW_RACING_LINE) ||
    (state_.intent == PlannerIntent::MERGE && proposed == PlannerIntent::FOLLOW_RACING_LINE) ||
    (state_.intent == PlannerIntent::FOLLOW_RACING_LINE && proposed == PlannerIntent::MERGE);
  return normal_fast ? TransitionClass::FAST : TransitionClass::SLOW;
}

TransitionReason RacingStateMachine::transitionReason(PlannerIntent proposed) const
{
  if (!state_.opponent.detected &&
    (state_.intent == PlannerIntent::OVERTAKE || state_.intent == PlannerIntent::PASS))
  {
    return TransitionReason::OPPONENT_LOST;
  }
  if (proposed == PlannerIntent::FOLLOW_RACING_LINE) {
    return TransitionReason::RACELINE_COMPATIBLE;
  }
  if (state_.intent == PlannerIntent::FOLLOW_RACING_LINE && proposed == PlannerIntent::MERGE) {
    return TransitionReason::RACELINE_DEPARTED;
  }
  if (proposed == PlannerIntent::OVERTAKE) {return TransitionReason::OPPONENT_ENGAGED;}
  if (state_.intent == PlannerIntent::MERGE && proposed == PlannerIntent::PASS) {
    return TransitionReason::OPPONENT_CLEARANCE_LOST;
  }
  if (proposed == PlannerIntent::PASS) {return TransitionReason::PASS_BAND_ENTERED;}
  if (proposed == PlannerIntent::MERGE) {return TransitionReason::OPPONENT_CLEARED;}
  return TransitionReason::STATE_CORRECTION;
}

bool RacingStateMachine::proposalUsesOpponent(PlannerIntent proposed) const
{
  if (state_.transition_reason == TransitionReason::OPPONENT_LOST) {return true;}
  return state_.opponent.detected && !(state_.intent == PlannerIntent::MERGE &&
         proposed == PlannerIntent::FOLLOW_RACING_LINE);
}

bool RacingStateMachine::confirmationSatisfied(PlannerIntent proposed) const
{
  const bool slow = state_.transition_class == TransitionClass::SLOW;
  const double required_s = slow ? config_.slow_confirmation_s : config_.fast_confirmation_s;
  if (state_.pending_duration_s + 1e-9 < required_s) {return false;}

  uint32_t required_grids = proposalUsesOpponent(proposed) ?
    config_.opponent_confirmation_grids : 0U;
  if (state_.intent == PlannerIntent::PASS && proposed == PlannerIntent::MERGE) {
    required_grids = config_.pass_merge_confirmation_grids;
    if (state_.merge_probe_valid_cycles < config_.merge_probe_confirmation_cycles) {
      return false;
    }
  }
  if (state_.intent == PlannerIntent::MERGE && proposed == PlannerIntent::PASS) {
    required_grids = config_.merge_pass_confirmation_grids;
  }
  return state_.pending_grid_count >= required_grids;
}

} // namespace local_planning
