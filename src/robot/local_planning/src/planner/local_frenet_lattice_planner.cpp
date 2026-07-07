#include "planning/planner/local_frenet_lattice_planner.hpp"

#include "planning/planner/collision_checker.hpp"
#include "planning/planner/edge_evaluator.hpp"
#include "planning/planner/path_processing.hpp"
#include "planning/planner/planner_costs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1e-6;
constexpr double kPi = 3.14159265358979323846;


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
  const Odometry & odom,
  const OccupancyGrid & grid,
  LocalPlannerIntent intent)
{
  LocalFrenetPlan result;

  if (frenet_converter_.getTotalLength() <= kEpsilon ||
    config_.layer_spacing_m <= kEpsilon ||
    config_.lane_spacing_m <= kEpsilon ||
    config_.sample_spacing_m <= kEpsilon ||
    config_.max_path_angle_deg <= 0.0 ||
    config_.max_path_angle_deg >= 90.0)
  {
    return result;
  }

  FrenetPoint start = frenet_converter_.cartesianToFrenet(odom.position);
  const std::vector<double> lanes = generateLaneOffsets();
  result.debug_lattice_lanes = generateDebugLatticeLanes(start, lanes);
  const int layer_count =
    std::max(1, static_cast<int>(std::ceil(config_.horizon_m / config_.layer_spacing_m)));
  const int lane_count = static_cast<int>(lanes.size());
  const int start_lane = nearestLaneIndex(start.d, lanes);
  const double heading_error = normalizeHeadingError(
    odom.heading - frenet_converter_.getRacingLineHeading(start.s));
  const double start_slope = std::clamp(std::tan(heading_error), -1.5, 1.5);
  start.slope = start_slope;

  std::vector<std::vector<DpState>> states(
    static_cast<size_t>(layer_count + 1),
    std::vector<DpState>(static_cast<size_t>(lane_count)));

  states[0][static_cast<size_t>(start_lane)].reachable = true;
  states[0][static_cast<size_t>(start_lane)].total_cost = 0.0;

  CollisionChecker collision_checker(config_);
  FrenetEdgeEvaluator edge_evaluator(config_, frenet_converter_, collision_checker);
  EdgeEvaluationScratch edge_scratch;

  /*
  i could defo make this clearer and use better practice
  its just harder for me to see on my screen if i nest a bunch of if's and get rid of the continues
  TODO: do that^

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
      const double max_slope = std::tan(config_.max_path_angle_deg * kPi / 180.0);

      for (int to_lane = 0; to_lane < lane_count; ++to_lane) {
        const double d_end = lanes[static_cast<size_t>(to_lane)];
        if (std::abs(d_end - d0) / config_.layer_spacing_m > max_slope) {
          continue;
        }

        EdgeEvaluation edge = edge_evaluator.evaluateEdge(
          s0, d0, slope0, d_end, 0.0, intent, grid, edge_scratch);

        if (edge.collision_status == CollisionStatus::COLLISION ||
          edge.collision_status == CollisionStatus::OUT_OF_GRID ||
          edge.collision_status == CollisionStatus::GEOMETRY_CONSTRAINT)
        {
          continue;
        }

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
        to_state.edge_samples.assign(edge_scratch.samples.begin(), edge_scratch.samples.end());
      }
    }
  }
  //at this point the dp table is actually filled
  /*
  what happens next you ask...
  1.  pick goal cell by scanning last layer and choose the lane with the lowest cost
      if we are in the merge state add the extra cost of merge_terminal_d_weight * d^2
  2.  if we need to tie break base it off of cost then curvature
  3.  if the last layer is blocked (obstacle or grid edge cut the lattice short)
      fall back to the deepest reachable layer, as long as the partial path is at
      least min_path_horizon_m long
  4.  and then yeah just reconstruct the path and return
  */

  const int min_goal_layer = std::max(
    1, static_cast<int>(std::ceil(config_.min_path_horizon_m / config_.layer_spacing_m)));

  int goal_layer = -1;
  int best_lane = -1;
  for (int layer = layer_count; layer >= min_goal_layer; --layer) {
    best_lane = selectBestLane(states, layer, lanes, intent);
    if (best_lane >= 0) {
      goal_layer = layer;
      break;
    }
  }

  if (best_lane < 0 || goal_layer < 0) {
    return result;
  }

  const SelectedLatticePath selected_path = reconstructSelectedPath(
    states, goal_layer, best_lane, lanes, start);
  result.path = selected_path.path;

  if (config_.angle_smoothing_enabled) {
    bool used_smoothed_path = false;
    result.path = smoothFrenetAnglesOrFallback(
      selected_path.anchors, selected_path.path, frenet_converter_, collision_checker, grid,
      config_, used_smoothed_path);
    if (used_smoothed_path) {
      assignVelocityLimitsFromGeometry(result.path);
    }
  }

  smoothVelocityProfile(result.path, odom.velocity, config_);
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

int LocalFrenetLatticePlanner::selectBestLane(
  const std::vector<std::vector<DpState>> & states,
  int layer,
  const std::vector<double> & lanes,
  LocalPlannerIntent intent) const
{
  int best_lane = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  double best_curvature_change = std::numeric_limits<double>::infinity();
  double best_intent_tie = std::numeric_limits<double>::infinity();
  for (int lane = 0; lane < static_cast<int>(lanes.size()); ++lane) {
    const DpState & state = states[static_cast<size_t>(layer)][static_cast<size_t>(lane)];
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

  return best_lane;
}

std::vector<std::vector<Point>> LocalFrenetLatticePlanner::generateDebugLatticeLanes(
  const FrenetPoint & start,
  const std::vector<double> & lanes) const
{
  std::vector<std::vector<Point>> lattice_lanes;
  if (lanes.empty() || config_.horizon_m <= kEpsilon || config_.layer_spacing_m <= kEpsilon) {
    return lattice_lanes;
  }

  const int sample_count = std::max(
    2, static_cast<int>(std::ceil(config_.horizon_m / config_.layer_spacing_m)) + 1);
  lattice_lanes.reserve(lanes.size());

  for (const double d : lanes) {
    std::vector<Point> lane;
    lane.reserve(static_cast<std::size_t>(sample_count));

    for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
      const double s = std::min(
        start.s + static_cast<double>(sample_index) * config_.layer_spacing_m,
        start.s + config_.horizon_m);
      lane.push_back(frenet_converter_.frenetToCartesian({s, d, 0.0}));
    }

    lattice_lanes.push_back(std::move(lane));
  }

  return lattice_lanes;
}

LocalFrenetLatticePlanner::SelectedLatticePath
LocalFrenetLatticePlanner::reconstructSelectedPath(
  const std::vector<std::vector<DpState>> & states,
  int final_layer,
  int final_lane,
  const std::vector<double> & lanes,
  const FrenetPoint & start) const
{
  SelectedLatticePath selected_path;
  if (states.size() < 2 || final_lane < 0 ||
    final_layer < 1 || final_layer >= static_cast<int>(states.size()))
  {
    return selected_path;
  }

  std::vector<const std::vector<Point> *> segments;
  segments.reserve(static_cast<size_t>(final_layer));
  size_t path_capacity = 0;
  std::vector<int> lane_by_layer(states.size(), -1);
  int lane = final_lane;
  for (int layer = final_layer; layer > 0; --layer) {
    if (lane < 0 || lane >= static_cast<int>(lanes.size())) {
      return {};
    }

    const DpState & state = states[static_cast<size_t>(layer)][static_cast<size_t>(lane)];
    if (!state.reachable) {
      return {};
    }

    segments.push_back(&state.edge_samples);
    path_capacity += state.edge_samples.size();
    lane_by_layer[static_cast<size_t>(layer)] = lane;
    lane = state.parent_lane;
    if (lane < 0) {
      return {};
    }
  }

  selected_path.path.reserve(path_capacity);
  for (auto segment_it = segments.rbegin(); segment_it != segments.rend(); ++segment_it) {
    const std::vector<Point> & segment = **segment_it;
    const size_t start_index = selected_path.path.empty() ? 0 : 1;
    for (size_t i = start_index; i < segment.size(); ++i) {
      selected_path.path.push_back(segment[i]);
    }
  }

  selected_path.anchors.reserve(static_cast<size_t>(final_layer) + 1);
  selected_path.anchors.push_back(start);
  for (std::size_t layer = 1; layer <= static_cast<std::size_t>(final_layer); ++layer) {
    const int layer_lane = lane_by_layer[layer];
    if (layer_lane < 0 || layer_lane >= static_cast<int>(lanes.size())) {
      return {};
    }

    selected_path.anchors.push_back(
      {
        start.s + static_cast<double>(layer) * config_.layer_spacing_m,
        lanes[static_cast<size_t>(layer_lane)],
        0.0
      });
  }

  return selected_path;
}

void LocalFrenetLatticePlanner::assignVelocityLimitsFromGeometry(std::vector<Point> & path) const
{
  if (path.empty()) {
    return;
  }

  std::vector<double> curvatures(path.size(), 0.0);
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    curvatures[i] = computeCurvature(path[i - 1], path[i], path[i + 1]);
  }
  if (path.size() > 2) {
    curvatures.front() = curvatures[1];
    curvatures.back() = curvatures[path.size() - 2];
  }

  for (std::size_t i = 0; i < path.size(); ++i) {
    const double s = frenet_converter_.cartesianToFrenet(path[i]).s;
    path[i].velocity = computeVelocity(s, curvatures[i], frenet_converter_, config_);
  }
}

} // namespace local_planning
