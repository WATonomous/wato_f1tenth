#include "planning/hybrid_astar_planner_code.hpp"

#include <algorithm>
#include <cmath>

namespace local_planning {

static constexpr double G = 9.81;

// Full Frenet curvature of the path
// κ_x = [δ·d'' + κ_r'·d·d' + κ_r·(δ² + 2d'²)] / (δ² + d'²)^(3/2)
// where δ = 1 - κ_r·d,  d' = dd/ds,  d'' = d²d/ds²
// brutal.
static double exactCurvature(const QuinticPolynomial& curve, double t,
                              double kappa_r, double kappa_r_prime) {
    double d     = curve.evaluate(t);
    double slope = curve.evaluateDerivative(t) / curve.delta_s;
    double dds2  = curve.evaluateSecondDerivative(t) / (curve.delta_s * curve.delta_s);
    double delta = 1.0 - kappa_r * d;
    double denom = delta * delta + slope * slope;
    return (delta * dds2 + kappa_r_prime * d * slope + kappa_r * (delta * delta + 2.0 * slope * slope))
           / std::pow(denom, 1.5);
}

double QuinticPolynomial::evaluate(double t) const {
    // a0 + t*(a1 + t*(a2 + t*(a3 + t*(a4 + t*a5))))
    return coeffs[0] + t * (coeffs[1] + t * (coeffs[2] + t * (coeffs[3] + t * (coeffs[4] + t * coeffs[5]))));
}

double QuinticPolynomial::evaluateDerivative(double t) const {
    // d'(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
    return coeffs[1] + t * (2.0 * coeffs[2] + t * (3.0 * coeffs[3] + t * (4.0 * coeffs[4] + t * 5.0 * coeffs[5])));
}

double QuinticPolynomial::evaluateSecondDerivative(double t) const {
    // d''(t) = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3
    //horner's method
    return 2.0 * coeffs[2] + t * (6.0 * coeffs[3] + t * (12.0 * coeffs[4] + t * 20.0 * coeffs[5]));
}

std::vector<double> QuinticPolynomial::sampleD(int num_points) const {
    std::vector<double> result(num_points);
    for (int i = 0; i < num_points; ++i) {
        double t;
        if(num_points > 1)
            t =  static_cast<double>(i) / (num_points - 1);
        else 
            t = 0.0;
        result[i] = evaluate(t);
    }
    return result;
}

//all values to be tuned later.
HybridAStarPlanner::HybridAStarPlanner()
    : lane_spacing_(0.1),
      layer_depth_(0.5),
      waypoint_spacing_(0.1), //waypoint_spacing_ is for OUTPUT path
      collision_check_spacing_(0.1),
      friction_coeff_(1.0) {} 

HybridAStarPlanner::~HybridAStarPlanner() = default;

void HybridAStarPlanner::setRacingLine(const std::vector<Point>& racing_line) {
    frenet_converter_.setRacingLine(racing_line);
}


// heading_start and heading_end are SLOPES.
//   d(0)   = d_start       →  a0 = d_start
//   d'(0)  = h_start*ds    →  a1 = h_start * delta_s  
//   d''(0) = 0             →  a2 = 0
//   d(1)   = d_end         →  a3 + a4 + a5 = D
//   d'(1)  = h_end*ds      →  3*a3 + 4*a4 + 5*a5 = H
//   d''(1) = 0             →  6*a3 + 12*a4 + 20*a5 = 0
//   where D = d_end - d_start - h_start*delta_s
//         H = (h_end - h_start) * delta_s
QuinticPolynomial HybridAStarPlanner::computeQuintic(
    double d_start, double heading_start,
    double d_end,   double heading_end,
    double delta_s) const
{
    double a0 = d_start;
    double a1 = heading_start * delta_s;
    double a2 = 0.0;

    double D = d_end - d_start - a1;
    double H = (heading_end - heading_start) * delta_s;

    double a3 = 10.0 * D - 4.0 * H;
    double a4 =  7.0 * H - 15.0 * D;
    double a5 =  6.0 * D -  3.0 * H;

    return {{a0, a1, a2, a3, a4, a5}, delta_s};
}

double HybridAStarPlanner::computeHeuristic(
    const LatticeNode& node,
    double s_goal) const
{
    // remaining distance along racing line
    return std::max(0.0, s_goal - node.frenet.s);
}

double HybridAStarPlanner::computeEdgeCost(
    const LatticeNode& from,
    const LatticeNode& to,
    const QuinticPolynomial& curve) const
{
    // cost weights (to be tuned)
    static constexpr double W_CURVATURE   = 1.0; //curve cost
    static constexpr double W_LATERAL     = 1.0; //distance out from racing line
    static constexpr double W_LANE_CHANGE = 1.0; //any lane change

    // sample curvature at 5 t values
    static constexpr int N = 5;
    double curvature_cost = 0.0;
    for (int i = 0; i < N; ++i) {
        double t      = static_cast<double>(i) / (N - 1);
        double s_at_t = from.frenet.s + t * curve.delta_s;
        double kr     = frenet_converter_.getRacingLineCurvature(s_at_t);
        double kr_p   = frenet_converter_.getRacingLineCurvatureDerivative(s_at_t);
        double kappa  = exactCurvature(curve, t, kr, kr_p);
        curvature_cost += kappa * kappa;
    }
    curvature_cost /= N;

    // prefer staying close to racing line (d=0)
    double lateral_cost = std::abs(to.frenet.d);

    // punish lane changes
    double lane_change_cost = 0.0;
    if (from.lane != to.lane) {
        lane_change_cost = 1.0;
    }

    return W_CURVATURE * curvature_cost + W_LATERAL * lateral_cost + W_LANE_CHANGE * lane_change_cost;
}

bool HybridAStarPlanner::shouldPrune(
    const LatticeNode& parent,
    const LatticeNode& /*child*/,
    const QuinticPolynomial& curve) const
{
    double v = frenet_converter_.getRacingLineVelocity(parent.frenet.s);
    // max lateral acceleration: (mu)*g = v²*κ  →  κ_max = μ*g / v²
    double kappa_max = friction_coeff_ * G / (v * v + 1e-3);

    //check N points and compare the curvature to the max curvature
    static constexpr int N = 5;
    for (int i = 0; i < N; ++i) {
        double t      = static_cast<double>(i) / (N - 1);
        double s_at_t = parent.frenet.s + t * curve.delta_s;
        double kr     = frenet_converter_.getRacingLineCurvature(s_at_t);
        double kr_p   = frenet_converter_.getRacingLineCurvatureDerivative(s_at_t);
        double kappa  = std::abs(exactCurvature(curve, t, kr, kr_p));
        if (kappa > kappa_max) {
            return true;
        }
    }
    return false;
}

double HybridAStarPlanner::computeVelocityAtPoint(double s, double curvature) const {
    double v_line = frenet_converter_.getRacingLineVelocity(s);
    if (std::abs(curvature) < 1e-7) {
        return v_line;
    }
    double v_dyn = std::sqrt(friction_coeff_ * G / std::abs(curvature));
    return std::min(v_line, v_dyn);
}

std::vector<Point> HybridAStarPlanner::plan(
    const Point& start_position,
    double start_heading,
    const Point& goal_position,
    const OccupancyGrid& grid)
{
    FrenetPoint start_frenet = frenet_converter_.cartesianToFrenet(start_position);
    FrenetPoint goal_frenet  = frenet_converter_.cartesianToFrenet(goal_position);
    double s_goal = goal_frenet.s;

    // heading_start for the first quintic must be a slope (dd/ds) not an angle.
    // dd/ds = tan(θ_error)
    // θ_error = ego heading - racing line heading at start point s
    double racing_line_heading = frenet_converter_.getRacingLineHeading(start_frenet.s);
    double start_heading_slope = std::tan(start_heading - racing_line_heading);

    LatticeNode start_node;
    start_node.layer        = 0;
    start_node.lane         = 0;
    start_node.frenet       = start_frenet;
    start_node.g_cost       = 0.0;
    start_node.h_cost       = computeHeuristic(start_node, s_goal);
    start_node.parent_layer = -1;
    start_node.parent_lane  = -1;

    return runAStar(start_node, start_heading_slope, s_goal, goal_position, grid);
}

std::vector<double> HybridAStarPlanner::generateLaneOffsets(
    double /*s*/,
    const OccupancyGrid& /*grid*/) const
{
    return {0.0};
}

bool HybridAStarPlanner::isCollisionFree(
    const QuinticPolynomial& /*curve*/,
    double /*s_start*/,
    const OccupancyGrid& /*grid*/) const
{
    return true;
}

std::vector<Point> HybridAStarPlanner::tryDirectShot(
    const LatticeNode& /*current*/,
    const Point& /*goal_position*/,
    double /*s_goal*/,
    const OccupancyGrid& /*grid*/) const
{
    return {};
}

std::vector<Point> HybridAStarPlanner::runAStar(
    const LatticeNode& /*start_node*/,
    double /*start_heading_slope*/,
    double /*s_goal*/,
    const Point& /*goal_position*/,
    const OccupancyGrid& /*grid*/)
{
    return {};
}

std::vector<Point> HybridAStarPlanner::reconstructPath(
    const std::vector<LatticeNode>& /*node_sequence*/) const
{
    return {};
}

std::vector<Point> HybridAStarPlanner::sampleQuintic(
    const QuinticPolynomial& /*curve*/,
    double /*s_start*/,
    double /*spacing*/) const
{
    return {};
}

} // namespace local_planning
