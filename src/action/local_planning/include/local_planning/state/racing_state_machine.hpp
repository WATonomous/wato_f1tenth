#ifndef LOCAL_PLANNING_STATE_RACING_STATE_MACHINE_HPP
#define LOCAL_PLANNING_STATE_RACING_STATE_MACHINE_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/reference/raceline_reference.hpp"

#include <cstdint>
#include <vector>

namespace local_planning
{

enum class RacingState : uint8_t
{
  STEADY_STATE = 0,
  BEHIND_OPPONENT = 1,
  SIDE_BY_SIDE = 2,
  AHEAD_OPPONENT = 3
};

struct OpponentState
{
  bool detected = false;
  double s = 0.0;
  double d = 0.0;
  Point position;
};

// Carried forward from the DP planner's state manager, projecting through
// RacelineReference.  It owns the ego seed across cycles; the opponent cell is
// projected with the heading-free overload seeded from ego, since a costmap
// cell has no orientation of its own.
//
// Phase 6 rewrites the transition logic against PRD 5: the four-intent output
// (FOLLOW / OVERTAKE / PASS / MERGE), the last-observation presence timeout,
// the merge completion dwell latch, and a detector that searches beside and
// behind ego rather than scanning only forward.
class RacingStateMachine
{
public:
  RacingStateMachine() = default;

  RacingStateMachine(RacingStateMachine &&) = delete;
  RacingStateMachine(const RacingStateMachine &) = delete;
  RacingStateMachine & operator=(const RacingStateMachine &) = delete;
  RacingStateMachine & operator=(RacingStateMachine &&) = delete;

  // Returns true when the state changed this update.
  bool update(const Odometry & ego_odom, const OccupancyGrid & occupancy_grid);

  RacingState getCurrentState() const {return current_state_;}
  OpponentState getOpponentState() const {return opponent_state_;}

  RacingState computeNextState(const Odometry & ego_odom);
  bool shouldAttemptOvertake(
    const Odometry & ego_odom,
    const OpponentState & opponent_state,
    double signed_gap_m) const;

  // Returns false if the point list is too degenerate to build a reference.
  bool setRacingLine(const std::vector<Point> & racing_line);
  void setTransitionConfig(
    double overtake_start_distance_m,
    double side_by_side_distance_m,
    double merge_start_gap_m,
    double merge_done_gap_m,
    double merge_done_d_m);

private:
  bool detectOpponentOnRacingLine(
    const OccupancyGrid & occupancy_grid,
    const Point & ego_position);

  // Signed distance along the raceline, positive when the opponent is ahead.
  double computeSignedDistanceToOpponent() const;

  RacingState current_state_ = RacingState::STEADY_STATE;
  RacingState previous_state_ = RacingState::STEADY_STATE;
  OpponentState opponent_state_;

  std::vector<Point> racing_line_;
  RacelineReference reference_;
  // Ego's projection seed, carried between cycles.  Stale-seed recovery inside
  // RacelineReference handles startup and relocalization.
  double ego_seed_s_ = 0.0;
  double ego_d_ = 0.0;

  double overtake_start_distance_m_ = 3.0;
  double side_by_side_distance_m_ = 0.5;
  double merge_start_gap_m_ = 1.0;
  double merge_done_gap_m_ = 2.0;
  double merge_done_d_m_ = 0.25;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_STATE_RACING_STATE_MACHINE_HPP
