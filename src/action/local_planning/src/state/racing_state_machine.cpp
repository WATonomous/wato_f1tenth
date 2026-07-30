#include "local_planning/state/racing_state_machine.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_planning
{
namespace
{

constexpr double kScanDistanceM = 10.0;
constexpr double kEpsilon = 1e-12;

} // namespace

void RacingStateMachine::setRacingLine(const std::vector<Point> & racing_line)
{
  racing_line_ = racing_line;
  cumulative_s_.clear();
  total_length_m_ = 0.0;

  if (racing_line_.size() < 2) {
    return;
  }

  cumulative_s_.resize(racing_line_.size());
  cumulative_s_[0] = 0.0;
  for (std::size_t i = 1; i < racing_line_.size(); ++i) {
    cumulative_s_[i] = cumulative_s_[i - 1] +
      std::hypot(
      racing_line_[i].x - racing_line_[i - 1].x,
      racing_line_[i].y - racing_line_[i - 1].y);
  }

  // Close the loop: the last waypoint back to the first.
  total_length_m_ = cumulative_s_.back() +
    std::hypot(
    racing_line_.front().x - racing_line_.back().x,
    racing_line_.front().y - racing_line_.back().y);
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

FrenetPoint RacingStateMachine::projectToRaceline(const Point & p) const
{
  if (racing_line_.size() < 2 || total_length_m_ <= kEpsilon) {
    return {};
  }

  const int n = static_cast<int>(racing_line_.size());
  double best_dist_sq = std::numeric_limits<double>::max();
  double best_s = 0.0;
  double best_d = 0.0;

  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const double ax = racing_line_[i].x;
    const double ay = racing_line_[i].y;
    const double abx = racing_line_[j].x - ax;
    const double aby = racing_line_[j].y - ay;
    const double seg_len_sq = abx * abx + aby * aby;
    if (seg_len_sq < kEpsilon) {
      continue;
    }

    const double apx = p.x - ax;
    const double apy = p.y - ay;
    const double t = std::clamp((apx * abx + apy * aby) / seg_len_sq, 0.0, 1.0);
    const double dx = p.x - (ax + t * abx);
    const double dy = p.y - (ay + t * aby);
    const double dist_sq = dx * dx + dy * dy;

    if (dist_sq < best_dist_sq) {
      const double seg_len = std::sqrt(seg_len_sq);
      best_dist_sq = dist_sq;
      best_s = cumulative_s_[static_cast<std::size_t>(i)] + t * seg_len;
      best_d = dx * (-aby / seg_len) + dy * (abx / seg_len);
    }
  }

  best_s = std::fmod(best_s, total_length_m_);
  if (best_s < 0.0) {
    best_s += total_length_m_;
  }
  return {best_s, best_d};
}

bool RacingStateMachine::update(
  const Odometry & ego_odom,
  const OccupancyGrid & occupancy_grid)
{
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

  const FrenetPoint ego_frenet = projectToRaceline(ego_odom.position);
  const double signed_gap_m = computeSignedDistanceToOpponent(ego_odom.position);

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
      if (signed_gap_m < -merge_done_gap_m_ &&
        std::abs(ego_frenet.d) < merge_done_d_m_)
      {
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
          const FrenetPoint fp = projectToRaceline(opponent_state_.position);
          opponent_state_.s = fp.s;
          opponent_state_.d = fp.d;
          return true;
        }
      }
    }
  }

  opponent_state_.detected = false;
  return false;
}

double RacingStateMachine::computeSignedDistanceToOpponent(const Point & ego_position) const
{
  const FrenetPoint ego_frenet = projectToRaceline(ego_position);
  double ds = opponent_state_.s - ego_frenet.s;

  // wrap to [-total/2, total/2] so positive = opponent ahead
  if (ds > total_length_m_ / 2.0) {
    ds -= total_length_m_;
  }
  if (ds < -total_length_m_ / 2.0) {
    ds += total_length_m_;
  }
  return ds;
}

} // namespace local_planning
