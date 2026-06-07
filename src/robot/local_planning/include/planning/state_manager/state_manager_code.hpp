#ifndef PLANNING_STATE_MANAGER_CODE_HPP
#define PLANNING_STATE_MANAGER_CODE_HPP

#include "planning/frenet_converter.hpp" // Point, Odometry, OccupancyGrid, FrenetPoint, FrenetConverter

#include <cstdint>
#include <vector>

namespace local_planning
{

// Racing states enum
enum class RacingState : uint8_t
{
  STEADY_STATE = 0,
  BEHIND_OPPONENT = 1,
  SIDE_BY_SIDE = 2,
  AHEAD_OPPONENT = 3
};

// Opponent state structure
struct OpponentState
{
  bool detected;
  double s;    // Frenet s-coordinate
  double d;    // Frenet d-coordinate
  Point position;
};

class RacingStateMachine
{
public:
  RacingStateMachine();
  //i remember behrad telling me to do the below i gotta do it for the rest of the classes too
  RacingStateMachine(const RacingStateMachine && other) = delete;  //move constructor disabled
  RacingStateMachine(const RacingStateMachine & other) = delete;  // copy constructor disabled
  RacingStateMachine & operator=(const RacingStateMachine &) = delete;
  RacingStateMachine & operator=(const RacingStateMachine &&) = delete;

  ~RacingStateMachine();

  //returns true when state updated
  bool update(
    const Odometry & ego_odom,
    const OccupancyGrid & occupancy_grid
    //const std::vector<Point>& racing_line //can add back if line gets updated live
  );

  // getters
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
    const Point & ego_position
  );

  // Compute signed distance along racing line (+ ahead, - behind opponent)
  double computeSignedDistanceToOpponent(const Point & ego_position) const;

  // states to track
  RacingState current_state_;
  RacingState previous_state_;
  std::vector<Point> racing_line_;
  OpponentState opponent_state_;

  double overtake_start_distance_m_;
  double side_by_side_distance_m_;
  double merge_start_gap_m_;
  double merge_done_gap_m_;
  double merge_done_d_m_;

  FrenetConverter frenet_converter_;
};

} // namespace local_planning

#endif // PLANNING_STATE_MANAGER_CODE_HPP
