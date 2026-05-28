#include "planning/planner/local_frenet_lattice_planner.hpp"

#include "planning/planner/collision_checker.hpp"
#include "planning/planner/edge_evaluator.hpp"
#include "planning/planner/planner_costs.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1e-6;

using SteadyClock = std::chrono::steady_clock;

double millisecondsSince(SteadyClock::time_point start)
{
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
}

} // namespace

void LocalFrenetLatticePlanner::setConfig(const LocalFrenetPlannerConfig & config)
{
  config_ = config;
}

void LocalFrenetLatticePlanner::setRacingLine(const std::vector<Point> & racing_line)
{
  frenet_converter_.setRacingLine(racing_line);
}

LocalFrenetPlan LocalFrenetLatticePlanner::plan(
  const Point & start_position,
  double start_heading,
  const OccupancyGrid & grid,
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
    config_.max_path_angle_deg >= 90.0)
  {
    result.diagnostics.planner_setup_ms = millisecondsSince(setup_start);
    return result;
  }

  const FrenetPoint start = frenet_converter_.cartesianToFrenet(start_position);
  const std::vector<double> lanes = generateLaneOffsets();
  const int layer_count =
    std::max(1, static_cast<int>(std::ceil(config_.horizon_m / config_.layer_spacing_m)));
  const int lane_count = static_cast<int>(lanes.size());
  const int start_lane = nearestLaneIndex(start.d, lanes);
  //wrap heading error
  const double heading_error = normalizeHeadingError(
    start_heading - frenet_converter_.getRacingLineHeading(
      start.s));

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

  CollisionChecker collision_checker(config_);
  FrenetEdgeEvaluator edge_evaluator(config_, frenet_converter_, collision_checker);

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
      const DpState & from_state =
        states[static_cast<size_t>(layer)][static_cast<size_t>(from_lane)];
      if (!from_state.reachable) {
        continue;
      }

      const double d0 = (layer == 0) ? start.d : lanes[static_cast<size_t>(from_lane)];
      const double slope0 = (layer == 0) ? start_slope : 0.0;

      for (int to_lane = 0; to_lane < lane_count; ++to_lane) {
        ++result.diagnostics.attempted_edges;
        EdgeEvaluation edge = edge_evaluator.evaluateEdge(
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
        DpState & to_state = states[static_cast<size_t>(next_layer)][static_cast<size_t>(to_lane)];
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
        to_state.curvature_change_cost = from_state.curvature_change_cost +
          edge.curvature_change_cost;
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
    const DpState & state = states[static_cast<size_t>(layer_count)][static_cast<size_t>(lane)];
    if (!state.reachable) {
      continue;
    }

    const double final_d = lanes[static_cast<size_t>(lane)];
    double total_cost = state.total_cost;
    if (intent == LocalPlannerIntent::MERGE) {
      total_cost += config_.merge_terminal_d_weight * final_d * final_d;
    }

    const double intent_tie = intentBias(final_d, intent, config_);
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
  const int max_index =
    std::max(
    0, static_cast<int>(std::floor(
      config_.max_lateral_offset_m / config_.lane_spacing_m)));
  std::vector<double> lanes;
  lanes.reserve(static_cast<size_t>(max_index * 2 + 1));
  for (int i = -max_index; i <= max_index; ++i) {
    lanes.push_back(static_cast<double>(i) * config_.lane_spacing_m);
  }
  return lanes;
}

int LocalFrenetLatticePlanner::nearestLaneIndex(double d, const std::vector<double> & lanes) const
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
  const std::vector<std::vector<DpState>> & states,
  int final_lane) const
{
  std::vector<std::vector<Point>> segments;
  int lane = final_lane;
  for (int layer = static_cast<int>(states.size()) - 1; layer > 0; --layer) {
    const DpState & state = states[static_cast<size_t>(layer)][static_cast<size_t>(lane)];
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
    const std::vector<Point> & segment = *segment_it;
    const size_t start_index = path.empty() ? 0 : 1;
    for (size_t i = start_index; i < segment.size(); ++i) {
      path.push_back(segment[i]);
    }
  }
  return path;
}

} // namespace local_planning
