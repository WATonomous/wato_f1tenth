#ifndef PLANNING_LOCAL_FRENET_LATTICE_PLANNER_HPP
#define PLANNING_LOCAL_FRENET_LATTICE_PLANNER_HPP

#include "frenet_converter.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace local_planning {

enum class LocalPlannerIntent : uint8_t {
    FOLLOW_RACING_LINE = 0,
    OVERTAKE = 1,
    MERGE = 2
};

std::string intentToString(LocalPlannerIntent intent);

//for explanations see the yaml
struct LocalFrenetPlannerConfig {
    double horizon_m = 6.0;
    double layer_spacing_m = 0.5;
    double lane_spacing_m = 0.1;
    double max_lateral_offset_m = 1.8;
    int max_lane_jump_per_layer = 3;
    double sample_spacing_m = 0.1;
    double obstacle_inflation_distance_m = 0.2;
    int occupied_threshold = 50;
    double friction_coeff = 1.0;
    double min_velocity_mps = 0.5;
    double max_velocity_mps = 10.0;
    double time_weight = 1.0;
    double curvature_change_weight = 0.4;
    double follow_d_weight = 0.20;
    double overtake_d_weight = 0.02;
    double merge_d_weight = 0.20;
    double merge_terminal_d_weight = 0.0;
};

//to make my debugging life easy
struct LocalFrenetPlannerDiagnostics {
    LocalPlannerIntent intent = LocalPlannerIntent::FOLLOW_RACING_LINE;
    FrenetPoint ego_frenet{};
    double selected_cost = 0.0;
    double selected_final_d = 0.0;
    int selected_final_lane = 0;
    int total_lanes = 0;
    int layers = 0;
    int valid_edges = 0;
    int invalid_collision_edges = 0;
    int invalid_out_of_grid_edges = 0;
    int invalid_dynamic_edges = 0;
    std::vector<double> lane_offsets;
};

struct LocalFrenetPlan {
    std::vector<Point> path;
    LocalFrenetPlannerDiagnostics diagnostics;
};

struct QuinticPolynomial {
    double coeffs[6]{};
    double delta_s = 0.0; //forward distance along the raceline for the corresponding edge

    double evaluate(double t) const;
    double evaluateDerivative(double t) const;
    double evaluateSecondDerivative(double t) const;
};

class LocalFrenetLatticePlanner {
public:
    void setConfig(const LocalFrenetPlannerConfig& config);
    void setRacingLine(const std::vector<Point>& racing_line);

    LocalFrenetPlan plan(
        const Point& start_position,
        double start_heading,
        const OccupancyGrid& grid,
        LocalPlannerIntent intent);

private:
    enum class CollisionStatus {
        FREE,
        COLLISION,
        OUT_OF_GRID
    };

    struct EdgeEvaluation {
        CollisionStatus collision_status = CollisionStatus::FREE;
        std::vector<Point> samples;
        double predicted_time_cost = 0.0;
        double curvature_change_cost = 0.0;
        double intent_bias_cost = 0.0; //how much we pull to raceline
        double total_cost = 0.0;
    };

    struct DpState {
        bool reachable = false;
        double total_cost = 0.0;
        double curvature_change_cost = 0.0;
        int parent_lane = -1;
        std::vector<Point> edge_samples; //x,y points along the corresponding edge
    };

    std::vector<double> generateLaneOffsets() const;
    QuinticPolynomial computeQuintic(
        double d_start,
        double slope_start,
        double d_end,
        double slope_end,
        double delta_s) const;

    EdgeEvaluation evaluateEdge(
        double s_start,
        double d_start,
        double slope_start,
        double d_end,
        LocalPlannerIntent intent,
        const OccupancyGrid& grid) const;

    CollisionStatus collisionStatus(const Point& p, double heading, const OccupancyGrid& grid) const;
    double computeCurvature(const Point& prev, const Point& curr, const Point& next) const;
    double computeVelocity(double s, double curvature) const;

    //custom cost based on the intent
    double intentBias(double d, LocalPlannerIntent intent) const;
    double normalizeHeadingError(double angle) const;
    int nearestLaneIndex(double d, const std::vector<double>& lanes) const;
    std::vector<Point> reconstructPath(
        const std::vector<std::vector<DpState>>& states,
        int final_lane) const;

    FrenetConverter frenet_converter_;
    LocalFrenetPlannerConfig config_;
};

} // namespace local_planning

#endif // PLANNING_LOCAL_FRENET_LATTICE_PLANNER_HPP
