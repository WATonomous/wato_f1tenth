#ifndef PLANNING_HYBRID_ASTAR_PLANNER_CODE_HPP
#define PLANNING_HYBRID_ASTAR_PLANNER_CODE_HPP

#include "frenet_converter.hpp" // FrenetConverter (types.hpp transitively: Point, OccupancyGrid, FrenetPoint)

#include <vector>

namespace local_planning {

// 2-State Lattice, can arrive at 2 diff headings, can be extended
enum class HeadingMode : uint8_t {
    PARALLEL = 0,       // subpath ends parallel to racing line
    GOAL_TARGETED = 1   // subpath points towards goal 
};

struct LatticeNode {
    int layer;
    int lane;
    FrenetPoint frenet;
    HeadingMode heading_mode;

    double g_cost;
    double h_cost;
    double f_cost() const {
        return g_cost + h_cost;
    }

    // parent tracking (-1 = no parent, i.e. start node)
    int parent_layer = -1;
    int parent_lane = -1;
    HeadingMode parent_heading_mode;
};

// d(t) = a0 + a1*t+ ...+ a5*t^5
// t normalized [0, 1] over delta_s
struct QuinticPolynomial {
    double coeffs[6];
    double delta_s;

    double evaluate(double t) const;
    double evaluateDerivative(double t) const;
    double evaluateSecondDerivative(double t) const;

    // sample 
    std::vector<double> sampleD(int num_points) const;
};

// main Planner

class HybridAStarPlanner {
public:
    HybridAStarPlanner();
    HybridAStarPlanner(const HybridAStarPlanner&) = delete;
    HybridAStarPlanner& operator=(const HybridAStarPlanner&) = delete;
    ~HybridAStarPlanner();

    // config
    void setRacingLine(const std::vector<Point>& racing_line);

    // Returns waypoints as Point(x, y, velocity) along quintic curves
    // return empty if no soln
    std::vector<Point> plan(
        const Point& start_position,
        double start_heading,
        const Point& goal_position,
        const OccupancyGrid& grid
    );

private:
    // generate d-offsets at given s by extending until hitting obstacles
    std::vector<double> generateLaneOffsets(
        double s,
        const OccupancyGrid& grid
    ) const;

    // fit quintic polynomial between two frenet states
    QuinticPolynomial computeQuintic(
        double d_start, double heading_start,
        double d_end, double heading_end,
        double delta_s
    ) const;

    double computeHeuristic(
        const LatticeNode& node,
        double s_goal
    ) const;


    double computeEdgeCost(
        const LatticeNode& from,
        const LatticeNode& to,
        const QuinticPolynomial& curve
    ) const;
    //cost of travering curve btwn 2 nodes
    //should punish curvature, lateral deviation, and lane changes


    // right now should implement a dynamics-based pruning
    // if curvature too big for speed, prune.
    bool shouldPrune(
        const LatticeNode& parent,
        const LatticeNode& child,
        const QuinticPolynomial& curve
    ) const;


    // checks if path collides with obstacles
    bool isCollisionFree(
        const QuinticPolynomial& curve,
        double s_start,
        const OccupancyGrid& grid
    ) const;

    // try one quintic from current node directly to s_goal, skipping all layers
    //  returns empty if blocked
    std::vector<Point> tryDirectShot(
        const LatticeNode& current,
        const Point& goal_position,
        double s_goal,
        const OccupancyGrid& grid
    ) const;

    // core A* search, returns final waypoint path
    // goal_position needed to compute goal-targeted heading at each expansion
    std::vector<Point> runAStar(
        const LatticeNode& start_node,
        double s_goal,
        const Point& goal_position,
        const OccupancyGrid& grid
    );

    // convert node sequence to waypoints
    std::vector<Point> reconstructPath(
        const std::vector<LatticeNode>& node_sequence
    ) const;

    // get a sample of points along quintic
    std::vector<Point> sampleQuintic(
        const QuinticPolynomial& curve,
        double s_start,
        double spacing
    ) const;

    // v = min(racing line vel, and dynamics limited velocity)
    double computeVelocityAtPoint(double s, double curvature) const;

    
    FrenetConverter frenet_converter_;

    // parameters
    double lane_spacing_;
    double layer_depth_;
    double waypoint_spacing_;
    double collision_check_spacing_;
    double friction_coeff_;
};

} // namespace local_planning

#endif // PLANNING_HYBRID_ASTAR_PLANNER_CODE_HPP