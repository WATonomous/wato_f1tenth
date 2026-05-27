#include "planning/local_frenet_lattice_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace local_planning {
namespace {

constexpr double kGravity = 9.81;
constexpr double kEpsilon = 1e-6;
constexpr double kPi = 3.14159265358979323846;
constexpr double kFrontCollisionCircleOffsetM = 0.26;

using SteadyClock = std::chrono::steady_clock;

double millisecondsSince(SteadyClock::time_point start)
{
    return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
}

double distance(const Point& a, const Point& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

} // namespace

std::string intentToString(LocalPlannerIntent intent)
{
    switch (intent) {
        case LocalPlannerIntent::FOLLOW_RACING_LINE:
            return "FOLLOW_RACING_LINE";
        case LocalPlannerIntent::OVERTAKE:
            return "OVERTAKE";
        case LocalPlannerIntent::MERGE:
            return "MERGE";
    }
    return "UNKNOWN";
} //just for logging

double QuinticPolynomial::evaluate(double t) const
{
    return coeffs[0] + t * (coeffs[1] + t * (coeffs[2] + t * (coeffs[3] + t * (coeffs[4] + t * coeffs[5]))));
}

double QuinticPolynomial::evaluateDerivative(double t) const
{
    return coeffs[1] + t * (2.0 * coeffs[2] + t * (3.0 * coeffs[3] + t * (4.0 * coeffs[4] + t * 5.0 * coeffs[5])));
}

double QuinticPolynomial::evaluateSecondDerivative(double t) const
{
    return 2.0 * coeffs[2] + t * (6.0 * coeffs[3] + t * (12.0 * coeffs[4] + t * 20.0 * coeffs[5]));
}

void LocalFrenetLatticePlanner::setConfig(const LocalFrenetPlannerConfig& config)
{
    config_ = config;
}

void LocalFrenetLatticePlanner::setRacingLine(const std::vector<Point>& racing_line)
{
    frenet_converter_.setRacingLine(racing_line);
}

LocalFrenetPlan LocalFrenetLatticePlanner::plan(
    const Point& start_position,
    double start_heading,
    const OccupancyGrid& grid,
    LocalPlannerIntent intent)
{
    LocalFrenetPlan result;
    result.diagnostics.intent = intent;

    const auto setup_start = SteadyClock::now();

    //catch any weird config issues 
    if (frenet_converter_.getTotalLength() <= kEpsilon ||
        config_.layer_spacing_m <= kEpsilon ||
        config_.lane_spacing_m <= kEpsilon ||
        config_.sample_spacing_m <= kEpsilon ||
        config_.max_path_angle_deg <= 0.0 ||
        config_.max_path_angle_deg >= 90.0) {
        result.diagnostics.planner_setup_ms = millisecondsSince(setup_start);
        return result;
    }

    const FrenetPoint start = frenet_converter_.cartesianToFrenet(start_position);
    const std::vector<double> lanes = generateLaneOffsets();
    const int layer_count = std::max(1, static_cast<int>(std::ceil(config_.horizon_m / config_.layer_spacing_m)));
    const int lane_count = static_cast<int>(lanes.size());
    const int start_lane = nearestLaneIndex(start.d, lanes);
    //wrap heading error
    const double heading_error = normalizeHeadingError(start_heading - frenet_converter_.getRacingLineHeading(start.s));
    
    //really large angles destroys the quintic
    //TODO: try out different values for clamping
    const double start_slope = std::clamp(std::tan(heading_error), -1.5, 1.5);

    //diagnostics
    result.diagnostics.ego_frenet = start;
    result.diagnostics.total_lanes = lane_count;
    result.diagnostics.layers = layer_count;
    result.diagnostics.lane_offsets = lanes;

    // states[layer][lane] eventually will add [theta] get excited i guess
    std::vector<std::vector<DpState>> states(
        static_cast<size_t>(layer_count + 1),
        std::vector<DpState>(static_cast<size_t>(lane_count)));

    states[0][static_cast<size_t>(start_lane)].reachable = true;
    states[0][static_cast<size_t>(start_lane)].total_cost = 0.0;
    result.diagnostics.planner_setup_ms = millisecondsSince(setup_start);

    const auto search_start = SteadyClock::now();

    /*
    i could defo make this clearer and use better practice
    its just harder for me to see on my screen if i nest a bunch of if's and get rid of the continues
    TODO: do that^
    but also im gonna rewrite a bunch of this when i start bucketting a bunch of angles anyway

    nodes: layer (s), lane (d)
    edges: only forward one layer with quintics connecting each hop
    update: just relax min cost like djikstra's normally does 
    layers give a fixed order so all we need is one forward pass
    time complexity is O(layers * lanes * samples * inflation_cells^2) i think 
    */
    for (int layer = 0; layer < layer_count; ++layer) {
        const double s0 = start.s + static_cast<double>(layer) * config_.layer_spacing_m;
        const int next_layer = layer + 1;

        for (int from_lane = 0; from_lane < lane_count; ++from_lane) {
            const DpState& from_state = states[static_cast<size_t>(layer)][static_cast<size_t>(from_lane)];
            if (!from_state.reachable) {
                continue;
            }

            const double d0 = (layer == 0) ? start.d : lanes[static_cast<size_t>(from_lane)];
            const double slope0 = (layer == 0) ? start_slope : 0.0;
            
            for (int to_lane = 0; to_lane < lane_count; ++to_lane) {
                ++result.diagnostics.attempted_edges;
                EdgeEvaluation edge = evaluateEdge(
                    s0, d0, slope0, lanes[static_cast<size_t>(to_lane)], intent, grid);

                if (edge.collision_status == CollisionStatus::COLLISION) {
                    ++result.diagnostics.invalid_collision_edges;
                    continue;
                }
                if (edge.collision_status == CollisionStatus::OUT_OF_GRID) {
                    ++result.diagnostics.invalid_out_of_grid_edges;
                    continue;
                }
                if (edge.collision_status == CollisionStatus::GEOMETRY_CONSTRAINT) {
                    ++result.diagnostics.invalid_geometry_edges;
                    continue;
                }

                ++result.diagnostics.valid_edges;
                //if we have a better total cost take it 
                //rn the tie break depends on curavture and not time but this could go either way
                //im not sure what makes more sense theoretically 
                DpState& to_state = states[static_cast<size_t>(next_layer)][static_cast<size_t>(to_lane)];
                const double new_total_cost = from_state.total_cost + edge.total_cost;
                const bool improves = !to_state.reachable ||
                    new_total_cost < to_state.total_cost - 1e-9 ||
                    (std::abs(new_total_cost - to_state.total_cost) < 1e-9 &&
                     edge.curvature_change_cost < to_state.curvature_change_cost);

                if (!improves) {
                    continue;
                }

                to_state.reachable = true;
                to_state.total_cost = new_total_cost;
                to_state.curvature_change_cost = from_state.curvature_change_cost + edge.curvature_change_cost;
                to_state.parent_lane = from_lane;
                to_state.edge_samples = std::move(edge.samples);
            }
        }
    }
    result.diagnostics.planner_search_ms = millisecondsSince(search_start);
    //at this point the dp table is actually filled
    /*
    what happens next you ask...
    1.  pick goal cell by scanning last layer and choose the lane with the lowest cost
        if we are in the merge state add the extra cost of merge_terminal_d_weight * d^2 
    2.  if we need to tie break base it off of cost then curvature
    3.  and then yeah just reconstruct the path and return 
    */

    const auto goal_select_start = SteadyClock::now();
    int best_lane = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    double best_curvature_change = std::numeric_limits<double>::infinity();
    double best_intent_tie = std::numeric_limits<double>::infinity();
    for (int lane = 0; lane < lane_count; ++lane) {
        const DpState& state = states[static_cast<size_t>(layer_count)][static_cast<size_t>(lane)];
        if (!state.reachable) {
            continue;
        }

        const double final_d = lanes[static_cast<size_t>(lane)];
        double total_cost = state.total_cost;
        if (intent == LocalPlannerIntent::MERGE) {
            total_cost += config_.merge_terminal_d_weight * final_d * final_d;
        }

        const double intent_tie = intentBias(final_d, intent);
        const bool better = total_cost < best_cost - 1e-9 ||
            (std::abs(total_cost - best_cost) < 1e-9 &&
             (state.curvature_change_cost < best_curvature_change - 1e-9 ||
              (std::abs(state.curvature_change_cost - best_curvature_change) < 1e-9 &&
               intent_tie < best_intent_tie - 1e-9)));

        if (better) {
            best_lane = lane;
            best_cost = total_cost;
            best_curvature_change = state.curvature_change_cost;
            best_intent_tie = intent_tie;
        }
    }

    result.diagnostics.planner_goal_select_ms = millisecondsSince(goal_select_start);

    if (best_lane < 0) {
        return result;
    }

    const auto reconstruct_start = SteadyClock::now();
    result.path = reconstructPath(states, best_lane);
    result.diagnostics.planner_reconstruct_ms = millisecondsSince(reconstruct_start);
    result.diagnostics.selected_cost = best_cost;
    result.diagnostics.selected_final_lane = best_lane;
    result.diagnostics.selected_final_d = lanes[static_cast<size_t>(best_lane)];
    return result;
}

//TODO: realistically this could be done to generate up to the wall on each side to save computations 
std::vector<double> LocalFrenetLatticePlanner::generateLaneOffsets() const
{
    const int max_index = std::max(0, static_cast<int>(std::floor(config_.max_lateral_offset_m / config_.lane_spacing_m)));
    std::vector<double> lanes;
    lanes.reserve(static_cast<size_t>(max_index * 2 + 1));
    for (int i = -max_index; i <= max_index; ++i) {
        lanes.push_back(static_cast<double>(i) * config_.lane_spacing_m);
    }
    return lanes;
}

QuinticPolynomial LocalFrenetLatticePlanner::computeQuintic(
    double d_start,
    double slope_start,
    double d_end,
    double slope_end,
    double delta_s) const
{
    const double a0 = d_start;
    const double a1 = slope_start * delta_s;
    const double a2 = 0.0;
    const double d = d_end - d_start - a1;
    const double h = (slope_end - slope_start) * delta_s;

    const double a3 = 10.0 * d - 4.0 * h;
    const double a4 = 7.0 * h - 15.0 * d;
    const double a5 = 6.0 * d - 3.0 * h;
    return {{a0, a1, a2, a3, a4, a5}, delta_s};
}



/*
    make sure the hop is valid and figure out its cost 
    1.  build quintic in d over layer spacing 
    2.  map it to x,y
    3.  reject if its out of grid space or collides with smth
        TODO:   it would be bad if there are tiny artifacts in the actual lidar
                that would trigger a fake collision
                this should probably be more robust
    4.  get curvature + velocity on samples and use these to get costs
    5. average out weight * d^2 on all the samples
    6. we return the cost and the samples 

*/
LocalFrenetLatticePlanner::EdgeEvaluation LocalFrenetLatticePlanner::evaluateEdge(
    double s_start,
    double d_start,
    double slope_start,
    double d_end,
    LocalPlannerIntent intent,
    const OccupancyGrid& grid) const
{
    EdgeEvaluation edge;
    const QuinticPolynomial curve = computeQuintic(
        d_start, slope_start, d_end, 0.0, config_.layer_spacing_m);
    const int sample_count = std::max(2, static_cast<int>(std::ceil(config_.layer_spacing_m / config_.sample_spacing_m)) + 1);
    const double max_path_angle_rad = config_.max_path_angle_deg * kPi / 180.0;

    edge.samples.reserve(static_cast<size_t>(sample_count));
    std::vector<double> curvatures(static_cast<size_t>(sample_count), 0.0);
    //get samples along quintic
    for (int i = 0; i < sample_count; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(sample_count - 1);
        const double s = s_start + t * config_.layer_spacing_m;
        Point p = frenet_converter_.frenetToCartesian({s, curve.evaluate(t)});

        const double path_slope = curve.evaluateDerivative(t) / curve.delta_s;
        const double path_angle = std::atan(path_slope);
        if (std::abs(path_angle) > max_path_angle_rad) {
            edge.collision_status = CollisionStatus::GEOMETRY_CONSTRAINT;
            return edge;
        }

        const double path_heading = frenet_converter_.getRacingLineHeading(s) + path_angle;
        const CollisionStatus status = collisionStatus(p, path_heading, grid);
        if (status != CollisionStatus::FREE) {
            edge.collision_status = status;
            return edge;
        }

        edge.samples.push_back(p);
    }
    //get all the curvatures 
    for (int i = 1; i + 1 < sample_count; ++i) {
        curvatures[static_cast<size_t>(i)] = computeCurvature(
            edge.samples[static_cast<size_t>(i - 1)],
            edge.samples[static_cast<size_t>(i)],
            edge.samples[static_cast<size_t>(i + 1)]);
    }
    if (sample_count > 2) {
        curvatures.front() = curvatures[1];
        curvatures.back() = curvatures[static_cast<size_t>(sample_count - 2)];
    }
    //get velocities across the samples
    for (int i = 0; i < sample_count; ++i) {
        const double s = s_start + (static_cast<double>(i) / static_cast<double>(sample_count - 1)) * config_.layer_spacing_m;
        edge.samples[static_cast<size_t>(i)].velocity = computeVelocity(s, curvatures[static_cast<size_t>(i)]);
    }
    /*
    the stuff below is just adding together a bunch of subcosts for the samples
    */
    for (int i = 1; i < sample_count; ++i) {
        const Point& prev = edge.samples[static_cast<size_t>(i - 1)];
        const Point& curr = edge.samples[static_cast<size_t>(i)];
        const double segment_length = distance(prev, curr);
        const double v = std::max(config_.min_velocity_mps, 0.5 * (prev.velocity + curr.velocity));
        edge.predicted_time_cost += segment_length / v;

        const double dk = curvatures[static_cast<size_t>(i)] - curvatures[static_cast<size_t>(i - 1)];
        edge.curvature_change_cost += dk * dk;
    }

    for (const Point& sample : edge.samples) {
        const FrenetPoint fp = frenet_converter_.cartesianToFrenet(sample);
        edge.intent_bias_cost += intentBias(fp.d, intent);
    }
    edge.intent_bias_cost /= static_cast<double>(edge.samples.size());

    edge.total_cost =
        config_.time_weight * edge.predicted_time_cost +
        config_.curvature_change_weight * edge.curvature_change_cost +
        edge.intent_bias_cost;
    return edge;
}

LocalFrenetLatticePlanner::CollisionStatus LocalFrenetLatticePlanner::collisionStatus(
    const Point& p,
    double heading,
    const OccupancyGrid& grid) const
{
    if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= kEpsilon) {
        return CollisionStatus::OUT_OF_GRID;
    }

    const int inflation_cells = std::max(0, static_cast<int>(std::ceil(config_.obstacle_inflation_distance_m / grid.resolution)));
    const double inflation_sq = config_.obstacle_inflation_distance_m * config_.obstacle_inflation_distance_m;
    const Point circle_centers[] = {
        p,
        {
            p.x + kFrontCollisionCircleOffsetM * std::cos(heading),
            p.y + kFrontCollisionCircleOffsetM * std::sin(heading),
            p.velocity
        }
    };

    for (const Point& center : circle_centers) {
        const int center_col = static_cast<int>((center.x - grid.origin.x) / grid.resolution);
        const int center_row = static_cast<int>((center.y - grid.origin.y) / grid.resolution);
        if (center_col < 0 || center_col >= grid.width || center_row < 0 || center_row >= grid.height) {
            return CollisionStatus::OUT_OF_GRID;
        }

        for (int dr = -inflation_cells; dr <= inflation_cells; ++dr) {
            for (int dc = -inflation_cells; dc <= inflation_cells; ++dc) {
                const int row = center_row + dr;
                const int col = center_col + dc;
                if (row < 0 || row >= grid.height || col < 0 || col >= grid.width) {
                    continue;
                }

                const double cell_x = grid.origin.x + (static_cast<double>(col) + 0.5) * grid.resolution;
                const double cell_y = grid.origin.y + (static_cast<double>(row) + 0.5) * grid.resolution;
                const double dx = cell_x - center.x;
                const double dy = cell_y - center.y;
                if (dx * dx + dy * dy > inflation_sq) {
                    continue;
                }
                if (grid.data[static_cast<size_t>(row * grid.width + col)] >= config_.occupied_threshold) {
                    return CollisionStatus::COLLISION;
                }
            }
        }
    }

    return CollisionStatus::FREE;
}

/*
    implements 3 point curvature formula basically find k = 1/R where R is the raidus of a circle through
    the 3 points
    the sign matters 
    positive = left
*/
double LocalFrenetLatticePlanner::computeCurvature(const Point& prev, const Point& curr, const Point& next) const
{
    const double a = distance(curr, next);
    const double b = distance(prev, next);
    const double c = distance(prev, curr);
    const double denom = a * b * c;
    if (denom < kEpsilon) {
        return 0.0;
    }

    const double cross =
        (curr.x - prev.x) * (next.y - prev.y) -
        (curr.y - prev.y) * (next.x - prev.x);
    return 2.0 * cross / denom;
}
//take min of raceline and physics limited, then clamp it between min and max from yaml
double LocalFrenetLatticePlanner::computeVelocity(double s, double curvature) const
{
    double velocity = std::max(config_.min_velocity_mps, frenet_converter_.getRacingLineVelocity(s));
    if (std::abs(curvature) > kEpsilon) {
        velocity = std::min(velocity, std::sqrt(config_.friction_coeff * kGravity / std::abs(curvature)));
    }
    if (config_.max_velocity_mps > config_.min_velocity_mps) {
        velocity = std::min(velocity, config_.max_velocity_mps);
    }
    return std::max(config_.min_velocity_mps, velocity);
}

//basically depending on the mode we punish the deviation from the racing line accordingly
double LocalFrenetLatticePlanner::intentBias(double d, LocalPlannerIntent intent) const
{
    switch (intent) {
        case LocalPlannerIntent::FOLLOW_RACING_LINE:
            return config_.follow_d_weight * d * d;
        case LocalPlannerIntent::OVERTAKE:
            return config_.overtake_d_weight * d * d;
        case LocalPlannerIntent::MERGE:
            return config_.merge_d_weight * d * d;
    }
    return config_.follow_d_weight * d * d;
}

//maps it to -pi to pi
//this is a quirky little trick 
double LocalFrenetLatticePlanner::normalizeHeadingError(double angle) const
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

int LocalFrenetLatticePlanner::nearestLaneIndex(double d, const std::vector<double>& lanes) const
{
    int best_index = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(lanes.size()); ++i) {
        const double candidate_distance = std::abs(d - lanes[static_cast<size_t>(i)]);
        if (candidate_distance < best_distance) {
            best_distance = candidate_distance;
            best_index = i;
        }
    }
    return best_index;
}

std::vector<Point> LocalFrenetLatticePlanner::reconstructPath(
    const std::vector<std::vector<DpState>>& states,
    int final_lane) const
{
    std::vector<std::vector<Point>> segments;
    int lane = final_lane;
    for (int layer = static_cast<int>(states.size()) - 1; layer > 0; --layer) {
        const DpState& state = states[static_cast<size_t>(layer)][static_cast<size_t>(lane)];
        if (!state.reachable) {
            return {};
        }
        segments.push_back(state.edge_samples);
        lane = state.parent_lane;
        if (lane < 0) {
            return {};
        }
    }

    std::vector<Point> path;
    for (auto segment_it = segments.rbegin(); segment_it != segments.rend(); ++segment_it) {
        const std::vector<Point>& segment = *segment_it;
        const size_t start_index = path.empty() ? 0 : 1;
        for (size_t i = start_index; i < segment.size(); ++i) {
            path.push_back(segment[i]);
        }
    }
    return path;
}

} // namespace local_planning
