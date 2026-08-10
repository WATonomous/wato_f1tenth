#include "local_planning/state/racing_state_machine.hpp"

#include <cmath>
#include <cstddef>
#include <utility>

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// Same constant and comparison as LocalPlannerConfig::occupied_threshold.
constexpr int8_t kOccupiedThreshold = 50;

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
  StateMachineConfig config)
: reference_(reference), config_(std::move(config))
{
}

void RacingStateMachine::update(
  const Odometry & ego_odom,
  const OccupancyGrid & occupancy_grid)
{
  if (!reference_.valid()) {
    state_ = TacticalState{};
    return;
  }

  // Ego first: its s seeds the next cycle and bounds the opponent scan.
  const Projection ego = reference_.project(
    ego_odom.position, ego_odom.heading, ego_seed_s_);
  ego_seed_s_ = ego.s;
  state_.ego_s = ego.s;
  state_.ego_d = ego.d;
  state_.ego_seed_was_stale = ego.seed_was_stale;

  state_.relative_position = detectOpponent(occupancy_grid, ego.s, state_.opponent) ?
    classify(state_.opponent.gap_m) :
    RelativePosition::NONE;

  state_.intent = nextIntent(ego_odom);
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
      occupancy_grid.data[index] >= kOccupiedThreshold)
    {
      return true;
    }
  }
  return false;
}

RelativePosition RacingStateMachine::classify(double gap_m) const
{
  if (gap_m >= config_.overlap_gap_m) {
    return RelativePosition::BEHIND;
  }
  if (gap_m > -config_.overlap_gap_m) {
    return RelativePosition::OVERLAPPING;
  }
  if (gap_m > -config_.clear_gap_m) {
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

  state_.raceline_compatible =
    std::abs(ego_d) <= config_.compat_lateral_m &&
    std::abs(state_.heading_error_rad) <= config_.compat_heading_rad;
  return state_.raceline_compatible;
}

PlannerIntent RacingStateMachine::nextIntent(const Odometry & ego_odom)
{
  const bool compatible = isRacelineCompatible(ego_odom, state_.ego_s, state_.ego_d);

  if (!state_.opponent.detected) {
    return compatible ? PlannerIntent::FOLLOW_RACING_LINE : PlannerIntent::MERGE;
  }

  switch (state_.relative_position) {
    case RelativePosition::NONE:   // unreachable; kept so the switch always returns
      return compatible ? PlannerIntent::FOLLOW_RACING_LINE : PlannerIntent::MERGE;

    // Shared intent on purpose: crossing this boundary mid-maneuver must not
    // produce an OVERTAKE -> MERGE jump.
    case RelativePosition::OVERLAPPING:
    case RelativePosition::AHEAD_NOT_CLEAR:
      return PlannerIntent::PASS;

    // Clear of the opponent: rejoin. A car already on the line has nothing to
    // merge back to, so it hands straight to the global follower rather than
    // planning a merge onto the station it is already at.
    case RelativePosition::AHEAD_AND_CLEAR:
      return compatible ? PlannerIntent::FOLLOW_RACING_LINE : PlannerIntent::MERGE;

    case RelativePosition::BEHIND:
      if (state_.opponent.gap_m < config_.overtake_start_gap_m) {
        return PlannerIntent::OVERTAKE;
      }
      return compatible ? PlannerIntent::FOLLOW_RACING_LINE : PlannerIntent::MERGE;
  }

  return PlannerIntent::MERGE;
}

} // namespace local_planning
