#ifndef PLANNING_STATE_MANAGER_CODE_HPP
#define PLANNING_STATE_MANAGER_CODE_HPP

#include "frenet_converter.hpp" // Point, Odometry, OccupancyGrid, FrenetPoint, FrenetConverter

#include <vector>

namespace local_planning {

// Racing states enum
enum class RacingState : uint8_t {
    STEADY_STATE = 0,
    BEHIND_OPPONENT = 1,
    SIDE_BY_SIDE = 2,
    AHEAD_OPPONENT = 3
};

// Opponent state structure
struct OpponentState {
    bool detected;
    double s;  // Frenet s-coordinate
    double d;  // Frenet d-coordinate
    Point position;
};

class RacingStateMachine {
public:
    RacingStateMachine();
    RacingStateMachine(const RacingStateMachine&& other) = delete; //move constructor disabled
    RacingStateMachine(const RacingStateMachine& other) = delete; // copy constructor disabled
    RacingStateMachine& operator=(const RacingStateMachine&) = delete;
    RacingStateMachine& operator=(const RacingStateMachine&&) = delete;

    ~RacingStateMachine();

    //returns true when state updated
    bool update(
        const Odometry& ego_odom,
        const OccupancyGrid& occupancy_grid,
        //const std::vector<Point>& racing_line //can add back if line gets updated live
    );

    // getters
    RacingState getCurrentState() const { return current_state_; }
    Point getCurrentGoal() const { return current_goal_; }
    OpponentState getOpponentState() const { return opponent_state_; }

    RacingState computeNextState(const Odometry& ego_odom);

private:
    void setRacingLine(const std::vector<Point>& racing_line); // called by constructor

    bool detectOpponentOnRacingLine(
        const OccupancyGrid& occupancy_grid,
        const Point& ego_position
    );

    // Compute signed distance along racing line (+ ahead, - behind opponent)
    double computeSignedDistanceToOpponent(const Point& ego_position) const;

    // Calculate target goal point based on current state
    Point calculateGoalPoint(const Odometry& ego_odom);

    // states to track
    RacingState current_state_;
    RacingState previous_state_;
    Point current_goal_;
    std::vector<Point> racing_line_;
    OpponentState opponent_state_;

    const double overtake_distance_;
    double lateral_tolerance_;
    double ahead_offset_;

    FrenetConverter frenet_converter_;
};

} // namespace local_planning

#endif // PLANNING_STATE_MANAGER_CODE_HPP
