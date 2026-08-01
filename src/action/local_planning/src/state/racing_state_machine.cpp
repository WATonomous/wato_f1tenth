#include "local_planning/state/racing_state_machine.hpp"

#include <cmath>
#include <limits>

namespace local_planning
{
namespace
{

constexpr double kScanDistanceM = 10.0;

} // namespace

bool RacingStateMachine::setRacingLine(const std::vector<Point> & racing_line)
{
  racing_line_ = racing_line;
  ego_seed_s_ = 0.0;
  ego_d_ = 0.0;
  return reference_.setRacingLine(racing_line);
}

void RacingStateMachine::setTransitionConfig(
  double overtake_start_distance_m,
  double side_by_side_distance_m,
  double merge_start_gap_m,
  double merge_done_gap_m,
  double merge_done_d_m)
{
  overtake_start_distance_m_ = overtake_start_distance_m;
  side_by_side_distance_m_ = side_by_side_distance_m;
  merge_start_gap_m_ = merge_start_gap_m;
  merge_done_gap_m_ = merge_done_gap_m;
  merge_done_d_m_ = merge_done_d_m;
}

bool RacingStateMachine::update(
  const Odometry & ego_odom,
  const OccupancyGrid & occupancy_grid)
{
  if (!reference_.valid()) {
    return false;
  }

  // Project ego first: its s seeds both the next cycle and the opponent search.
  const Projection ego = reference_.project(
    ego_odom.position, ego_odom.heading, ego_seed_s_);
  ego_seed_s_ = ego.s;
  ego_d_ = ego.d;

  previous_state_ = current_state_;
  detectOpponentOnRacingLine(occupancy_grid, ego_odom.position);
  current_state_ = computeNextState(ego_odom);
  return current_state_ != previous_state_;
}

RacingState RacingStateMachine::computeNextState(const Odometry & ego_odom)
{
  if (!opponent_state_.detected) {
    return RacingState::STEADY_STATE;
  }

  const double signed_gap_m = computeSignedDistanceToOpponent();

  switch (current_state_) {
    case RacingState::STEADY_STATE:
      if (shouldAttemptOvertake(ego_odom, opponent_state_, signed_gap_m)) {
        return RacingState::BEHIND_OPPONENT;
      }
      return RacingState::STEADY_STATE;

    case RacingState::BEHIND_OPPONENT:
      if (signed_gap_m < -merge_start_gap_m_) {
        return RacingState::AHEAD_OPPONENT;
      }
      if (std::abs(signed_gap_m) < side_by_side_distance_m_) {
        return RacingState::SIDE_BY_SIDE;
      }
      return RacingState::BEHIND_OPPONENT;

    case RacingState::SIDE_BY_SIDE:
      if (signed_gap_m < -merge_start_gap_m_) {
        return RacingState::AHEAD_OPPONENT;
      }
      return RacingState::SIDE_BY_SIDE;

    case RacingState::AHEAD_OPPONENT:
      if (signed_gap_m < -merge_done_gap_m_ && std::abs(ego_d_) < merge_done_d_m_) {
        return RacingState::STEADY_STATE;
      }
      return RacingState::AHEAD_OPPONENT;
  }

  return RacingState::STEADY_STATE;
}

/*
this logic should eventually account for if we are actually gaining on the ego
its silly to overtake if we are
i would do it like sample a bunch of delta s values and if our s is getting closer to theirs
switch to overtaking treat it like an extra condition
*/
bool RacingStateMachine::shouldAttemptOvertake(
  const Odometry & /*ego_odom*/,
  const OpponentState & opponent_state,
  double signed_gap_m) const
{
  return opponent_state.detected &&
         signed_gap_m > 0.0 &&
         signed_gap_m < overtake_start_distance_m_;
}

bool RacingStateMachine::detectOpponentOnRacingLine(
  const OccupancyGrid & occupancy_grid,
  const Point & ego_position)
{
  if (racing_line_.empty()) {
    opponent_state_.detected = false;
    return false;
  }

  const int n = static_cast<int>(racing_line_.size());

  int ego_idx = 0;
  double best_dist_sq = std::numeric_limits<double>::max();
  for (int i = 0; i < n; ++i) {
    const double dx = ego_position.x - racing_line_[i].x;
    const double dy = ego_position.y - racing_line_[i].y;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      ego_idx = i;
    }
  }

  // scan forward along racing line
  double accumulated = 0.0;
  for (int step = 1; accumulated < kScanDistanceM; ++step) {
    const int curr = (ego_idx + step) % n;
    const int prev = (ego_idx + step - 1) % n;

    const double dx = racing_line_[curr].x - racing_line_[prev].x;
    const double dy = racing_line_[curr].y - racing_line_[prev].y;
    accumulated += std::hypot(dx, dy);

    const int col =
      static_cast<int>((racing_line_[curr].x - occupancy_grid.origin.x) /
      occupancy_grid.resolution);
    const int row =
      static_cast<int>((racing_line_[curr].y - occupancy_grid.origin.y) /
      occupancy_grid.resolution);

    // check 3x3 neighborhood around this waypoint (15 cm square basically)
    //this should be changed to use the size of the square because we might change
    //the dimensions of the costmap and it would fuck this part completely
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        const int r = row + dr;
        const int c = col + dc;
        if (r < 0 || r >= occupancy_grid.height || c < 0 || c >= occupancy_grid.width) {
          continue;
        }
        if (occupancy_grid.data[static_cast<std::size_t>(r * occupancy_grid.width + c)] > 50) {
          opponent_state_.detected = true;
          opponent_state_.position = Point(
            occupancy_grid.origin.x + (c + 0.5) * occupancy_grid.resolution,
            occupancy_grid.origin.y + (r + 0.5) * occupancy_grid.resolution);
          // A costmap cell has no heading, so the seed window rather than a
          // tangent check is what keeps this on the right branch.  Seeding from
          // ego is sound because the scan only reaches kScanDistanceM ahead.
          const Projection opponent =
            reference_.project(opponent_state_.position, ego_seed_s_);
          opponent_state_.s = opponent.s;
          opponent_state_.d = opponent.d;
          return true;
        }
      }
    }
  }

  opponent_state_.detected = false;
  return false;
}

double RacingStateMachine::computeSignedDistanceToOpponent() const
{
  // Wrap-aware and signed: positive means the opponent is ahead of ego.
  return reference_.deltaS(ego_seed_s_, opponent_state_.s);
}

} // namespace local_planning
