#ifndef PLANNING_STATE_MANAGER_CODE_HPP
#define PLANNING_STATE_MANAGER_CODE_HPP

#include <vector>
#include <memory>

namespace local_planning {

struct Point {
    double x_;
    double y_;
    double velocity_;

    Point() : x_(0.0), y_(0.0), velocity_(0.0) {}
    Point(double x = 0.0, double y = 0.0, double velocity = 0.0) : x_(x), y_(y), velocity_(velocity) {}
};

struct Odometry {
    Point position;
    double velocity;
    double heading;
};

struct OccupancyGrid {
    std::vector<int8_t> data; //  -1 (unknown), otherwise 0 to 100 for probability occupied
    int width;
    int height;
    double resolution;
    Point origin;
};

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
    
    Point findClosestRacingLinePoint(const Point& position) const;
    
    int findClosestRacingLineIndex(const Point& position) const;
    
    //deal with a closed loop track
    double wrapS(double s) const;

    
    // covert x,y to frenet frame 
    void projectToFrenet(const Point& position, double& s, double& d) const;
    
    Point getRacingLinePoint(double s) const;

    // Build prefix sum array of distances between waypoints
    void buildDistancePrefixSum();
    
    // states to track 
    RacingState current_state_;
    RacingState previous_state_;
    Point current_goal_;
    std::vector<Point> racing_line_;
    OpponentState opponent_state_;
    
    const double overtake_distance_;
    double lateral_tolerance_;
    double ahead_offset_;

    
    std::vector<double> waypoint_distance_prefix_sum_; // total distance from start to waypoint i
    double total_track_length_;
    
};

} // namespace local_planning

#endif // PLANNING_STATE_MANAGER_CODE_HPP