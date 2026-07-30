#ifndef LOCAL_PLANNING_STATE_RACING_STATE_MACHINE_HPP
#define LOCAL_PLANNING_STATE_RACING_STATE_MACHINE_HPP

#include "local_planning/core/types.hpp"

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

// Carried forward from the DP planner's state manager, with FrenetConverter
// swapped for the local polyline projection below so nothing in this package
// depends on the deleted converter.
//
// Phase 6 rewrites the transition logic against PRD 5: the four-intent output
// (FOLLOW / OVERTAKE / PASS / MERGE), the last-observation presence timeout,
// the merge completion dwell latch, and a detector that searches beside and
// behind ego rather than scanning only forward.  The projection here becomes a
// RacelineReference call once Phase 1 lands.
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

  void setRacingLine(const std::vector<Point> & racing_line);
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

  // Globally-nearest projection onto the closed raceline polyline.  Known to
  // be branch-unsafe at a hairpin; PRD 24 replaces it with the locally-seeded,
  // tangent-checked RacelineReference projection in Phase 1.
  FrenetPoint projectToRaceline(const Point & p) const;

  // Signed distance along the raceline, positive when the opponent is ahead.
  double computeSignedDistanceToOpponent(const Point & ego_position) const;

  RacingState current_state_ = RacingState::STEADY_STATE;
  RacingState previous_state_ = RacingState::STEADY_STATE;
  OpponentState opponent_state_;

  std::vector<Point> racing_line_;
  // cumulative_s_[i] is the polyline arc length from waypoint 0 to waypoint i.
  std::vector<double> cumulative_s_;
  double total_length_m_ = 0.0;

  double overtake_start_distance_m_ = 3.0;
  double side_by_side_distance_m_ = 0.5;
  double merge_start_gap_m_ = 1.0;
  double merge_done_gap_m_ = 2.0;
  double merge_done_d_m_ = 0.25;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_STATE_RACING_STATE_MACHINE_HPP
