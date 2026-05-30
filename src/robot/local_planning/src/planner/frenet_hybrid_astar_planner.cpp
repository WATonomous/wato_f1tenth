#include "planning/planner/frenet_hybrid_astar_planner.hpp"

#include "planning/planner/collision_checker.hpp"
#include "planning/planner/edge_evaluator.hpp"
#include "planning/planner/planner_costs.hpp"
#include "planning/planner/quintic_polynomial.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-6;
constexpr double kTieEpsilon = 1e-9;
constexpr double kLargeHeuristic = 1.0e6;
constexpr int kEmergencyExpansionCap = 1000000;

using SteadyClock = std::chrono::steady_clock;

double millisecondsSince(SteadyClock::time_point start)
{
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
}

double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

double radiansToDegrees(double radians)
{
  return radians * 180.0 / kPi;
}

} // namespace

void FrenetHybridAStarPlanner::setConfig(const LocalFrenetPlannerConfig & config)
{
  config_ = config;
}

void FrenetHybridAStarPlanner::setRacingLine(const std::vector<Point> & racing_line)
{
  frenet_converter_.setRacingLine(racing_line);
}

LocalFrenetPlan FrenetHybridAStarPlanner::plan(
  const Point & start_position,
  double start_heading,
  const OccupancyGrid & grid,
  LocalPlannerIntent intent)
{
  LocalFrenetPlan result;
  result.diagnostics.intent = intent;

  const auto setup_start = SteadyClock::now();

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
  const std::vector<double> heading_buckets = generateHeadingBucketsRad();
  if (lanes.empty() || heading_buckets.empty()) {
    result.diagnostics.planner_setup_ms = millisecondsSince(setup_start);
    return result;
  }

  const int final_layer =
    std::max(1, static_cast<int>(std::ceil(config_.horizon_m / config_.layer_spacing_m)));
  const int lane_count = static_cast<int>(lanes.size());
  const int heading_count = static_cast<int>(heading_buckets.size());
  const int start_lane = nearestLaneIndex(start.d, lanes);
  const double heading_error = normalizeHeadingError(
    start_heading - frenet_converter_.getRacingLineHeading(start.s));
  const int start_heading_idx = nearestHeadingIndex(heading_error, heading_buckets);
  const double start_slope = headingToSlope(heading_error);

  result.diagnostics.ego_frenet = start;
  result.diagnostics.total_lanes = lane_count;
  result.diagnostics.total_heading_buckets = heading_count;
  result.diagnostics.layers = final_layer;
  result.diagnostics.lane_offsets = lanes;

  const int state_count = (final_layer + 1) * lane_count * heading_count;
  std::vector<SearchState> states(static_cast<size_t>(state_count));
  for (int layer = 0; layer <= final_layer; ++layer) {
    for (int lane = 0; lane < lane_count; ++lane) {
      for (int heading = 0; heading < heading_count; ++heading) {
        SearchState & state = states[static_cast<size_t>(stateIndex(
              layer, lane, heading, lane_count, heading_count))];
        state.layer_idx = layer;
        state.lane_idx = lane;
        state.heading_idx = heading;
        state.g_cost = std::numeric_limits<double>::infinity();
        state.h_cost = 0.0;
        state.f_cost = std::numeric_limits<double>::infinity();
        state.parent_index = -1;
      }
    }
  }

  CollisionChecker collision_checker(config_);
  FrenetEdgeEvaluator edge_evaluator(config_, frenet_converter_, collision_checker);
  EdgeEvaluationScratch edge_scratch;

  const int start_index = stateIndex(
    0, start_lane, start_heading_idx, lane_count, heading_count);
  SearchState & start_state = states[static_cast<size_t>(start_index)];
  start_state.reached = true;
  start_state.g_cost = 0.0;
  start_state.h_cost = heuristicShotTime(0, start.s, start.d, heading_error, intent, final_layer);
  start_state.f_cost = config_.heuristic_weight * start_state.h_cost;

  auto compare_queue_items = [](const QueueItem & lhs, const QueueItem & rhs) {
      if (std::abs(lhs.f_cost - rhs.f_cost) > kTieEpsilon) {
        return lhs.f_cost > rhs.f_cost;
      }
      return lhs.state_index > rhs.state_index;
    };
  std::priority_queue<QueueItem, std::vector<QueueItem>, decltype(compare_queue_items)> open(
    compare_queue_items);
  open.push({start_state.f_cost, start_index});
  result.diagnostics.states_pushed = 1;

  result.diagnostics.planner_setup_ms = millisecondsSince(setup_start);

  const auto search_start = SteadyClock::now();
  const double runtime_budget_ms = std::max(0.0, config_.max_runtime_ms);
  auto runtimeExpired = [&]() {
      return millisecondsSince(search_start) >= runtime_budget_ms;
    };

  int best_partial_index = start_index;
  std::vector<int> final_candidates;
  int expansions = 0;
  bool stop_search = false;

  while (!open.empty() && !runtimeExpired()) {
    if (expansions >= kEmergencyExpansionCap) {
      break;
    }

    const QueueItem item = open.top();
    open.pop();

    SearchState & current = states[static_cast<size_t>(item.state_index)];
    if (current.closed || item.f_cost > current.f_cost + kTieEpsilon) {
      continue;
    }

    current.closed = true;
    ++expansions;
    ++result.diagnostics.states_popped;

    if (isBetterPartial(current, states[static_cast<size_t>(best_partial_index)])) {
      best_partial_index = item.state_index;
    }
    result.diagnostics.deepest_layer_reached = std::max(
      result.diagnostics.deepest_layer_reached, current.layer_idx);

    if (current.layer_idx == final_layer) {
      continue;
    }

    const int next_layer = current.layer_idx + 1;
    const double s0 = start.s + static_cast<double>(current.layer_idx) * config_.layer_spacing_m;
    const double d0 =
      (current.layer_idx == 0) ? start.d : lanes[static_cast<size_t>(current.lane_idx)];
    const double theta0 = (item.state_index == start_index) ?
      heading_error : heading_buckets[static_cast<size_t>(current.heading_idx)];
    const double slope0 = (item.state_index == start_index) ? start_slope : headingToSlope(theta0);

    const int max_lane_jump = std::max(0, config_.max_lane_jump_per_layer);
    const int min_lane = std::max(0, current.lane_idx - max_lane_jump);
    const int max_lane = std::min(lane_count - 1, current.lane_idx + max_lane_jump);
    const int max_heading_jump = std::max(0, config_.max_heading_jump_per_layer);
    const int min_heading = std::max(0, current.heading_idx - max_heading_jump);
    const int max_heading = std::min(heading_count - 1, current.heading_idx + max_heading_jump);

    for (int next_lane = min_lane; next_lane <= max_lane && !stop_search; ++next_lane) {
      const double d1 = lanes[static_cast<size_t>(next_lane)];
      for (int next_heading = min_heading; next_heading <= max_heading; ++next_heading) {
        if (runtimeExpired()) {
          stop_search = true;
          break;
        }

        const int successor_index = stateIndex(
          next_layer, next_lane, next_heading, lane_count, heading_count);
        SearchState & successor = states[static_cast<size_t>(successor_index)];
        if (successor.closed) {
          continue;
        }

        const double theta1 = heading_buckets[static_cast<size_t>(next_heading)];
        const double slope1 = headingToSlope(theta1);
        ++result.diagnostics.attempted_edges;

        if (!headingTransitionFeasible(d0, d1, theta0, theta1)) {
          ++result.diagnostics.invalid_geometry_edges;
          continue;
        }

        EdgeEvaluation edge = edge_evaluator.evaluateEdge(
          s0, d0, slope0, d1, slope1, intent, grid, edge_scratch);

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
        const double tentative_g = current.g_cost + edge.total_cost;
        const double tentative_curvature_change =
          current.curvature_change_cost + edge.curvature_change_cost;
        const bool improves = !successor.reached ||
          tentative_g < successor.g_cost - kTieEpsilon ||
          (std::abs(tentative_g - successor.g_cost) < kTieEpsilon &&
          tentative_curvature_change < successor.curvature_change_cost - kTieEpsilon);
        if (!improves) {
          continue;
        }

        successor.reached = true;
        successor.g_cost = tentative_g;
        successor.h_cost = heuristicShotTime(
          next_layer,
          start.s + static_cast<double>(next_layer) * config_.layer_spacing_m,
          d1,
          theta1,
          intent,
          final_layer);
        successor.f_cost = successor.g_cost + config_.heuristic_weight * successor.h_cost;
        successor.curvature_change_cost = tentative_curvature_change;
        successor.parent_index = item.state_index;
        successor.edge_samples.assign(edge_scratch.samples.begin(), edge_scratch.samples.end());

        if (isBetterPartial(successor, states[static_cast<size_t>(best_partial_index)])) {
          best_partial_index = successor_index;
        }
        result.diagnostics.deepest_layer_reached = std::max(
          result.diagnostics.deepest_layer_reached, successor.layer_idx);

        if (next_layer == final_layer && !successor.final_candidate_recorded) {
          successor.final_candidate_recorded = true;
          final_candidates.push_back(successor_index);
        }

        open.push({successor.f_cost, successor_index});
        ++result.diagnostics.states_pushed;
      }
    }

    if (stop_search) {
      break;
    }
  }

  result.diagnostics.planner_search_ms = millisecondsSince(search_start);
  result.diagnostics.runtime_ms = result.diagnostics.planner_search_ms;
  result.diagnostics.final_candidates_found = static_cast<int>(final_candidates.size());

  const auto goal_select_start = SteadyClock::now();
  int selected_index = -1;
  bool returned_partial = false;

  auto candidateCost = [&](int state_index) {
      const SearchState & state = states[static_cast<size_t>(state_index)];
      double cost = state.g_cost;
      if (intent == LocalPlannerIntent::MERGE && state.layer_idx == final_layer) {
        const double d = lanes[static_cast<size_t>(state.lane_idx)];
        cost += config_.merge_terminal_d_weight * d * d;
      }
      return cost;
    };

  for (int candidate_index : final_candidates) {
    if (!states[static_cast<size_t>(candidate_index)].reached) {
      continue;
    }
    if (selected_index < 0 ||
      candidateCost(candidate_index) < candidateCost(selected_index) - kTieEpsilon ||
      (std::abs(candidateCost(candidate_index) - candidateCost(selected_index)) < kTieEpsilon &&
      states[static_cast<size_t>(candidate_index)].curvature_change_cost <
      states[static_cast<size_t>(selected_index)].curvature_change_cost - kTieEpsilon))
    {
      selected_index = candidate_index;
    }
  }

  if (selected_index < 0 && best_partial_index != start_index) {
    selected_index = best_partial_index;
    returned_partial = true;
  }

  result.diagnostics.planner_goal_select_ms = millisecondsSince(goal_select_start);

  if (selected_index < 0) {
    return result;
  }

  const auto reconstruct_start = SteadyClock::now();
  result.path = reconstructPath(states, selected_index);
  result.diagnostics.planner_reconstruct_ms = millisecondsSince(reconstruct_start);

  const SearchState & selected = states[static_cast<size_t>(selected_index)];
  const double selected_d = (selected.layer_idx == 0) ?
    start.d : lanes[static_cast<size_t>(selected.lane_idx)];
  result.diagnostics.selected_cost = candidateCost(selected_index);
  result.diagnostics.selected_g_cost = selected.g_cost;
  result.diagnostics.selected_h_cost = selected.h_cost;
  result.diagnostics.selected_f_cost = selected.f_cost;
  result.diagnostics.selected_final_lane = selected.lane_idx;
  result.diagnostics.selected_final_d = selected_d;
  result.diagnostics.selected_final_heading_idx = selected.heading_idx;
  result.diagnostics.selected_final_heading_deg = radiansToDegrees(
    heading_buckets[static_cast<size_t>(selected.heading_idx)]);
  result.diagnostics.returned_partial_path = returned_partial;

  return result;
}

std::vector<double> FrenetHybridAStarPlanner::generateLaneOffsets() const
{
  const int max_index = std::max(
    0,
    static_cast<int>(std::floor(config_.max_lateral_offset_m / config_.lane_spacing_m)));
  std::vector<double> lanes;
  lanes.reserve(static_cast<size_t>(max_index * 2 + 1));
  for (int i = -max_index; i <= max_index; ++i) {
    lanes.push_back(static_cast<double>(i) * config_.lane_spacing_m);
  }
  return lanes;
}

std::vector<double> FrenetHybridAStarPlanner::generateHeadingBucketsRad() const
{
  std::vector<double> headings;
  if (config_.heading_buckets_deg.empty()) {
    headings.push_back(0.0);
    return headings;
  }

  headings.reserve(config_.heading_buckets_deg.size());
  for (double heading_deg : config_.heading_buckets_deg) {
    headings.push_back(degreesToRadians(heading_deg));
  }
  return headings;
}

int FrenetHybridAStarPlanner::nearestLaneIndex(
  double d,
  const std::vector<double> & lanes) const
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

int FrenetHybridAStarPlanner::nearestHeadingIndex(
  double theta_rel,
  const std::vector<double> & heading_buckets) const
{
  int best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(heading_buckets.size()); ++i) {
    const double candidate_distance = std::abs(
      normalizeHeadingError(
        theta_rel - heading_buckets[static_cast<size_t>(i)]));
    if (candidate_distance < best_distance) {
      best_distance = candidate_distance;
      best_index = i;
    }
  }
  return best_index;
}

int FrenetHybridAStarPlanner::stateIndex(
  int layer_idx,
  int lane_idx,
  int heading_idx,
  int lane_count,
  int heading_count) const
{
  return ((layer_idx * lane_count) + lane_idx) * heading_count + heading_idx;
}

double FrenetHybridAStarPlanner::headingToSlope(double theta_rel) const
{
  const double max_angle = degreesToRadians(std::min(85.0, std::abs(config_.max_path_angle_deg)));
  const double clamped_theta = std::clamp(theta_rel, -max_angle, max_angle);
  return std::tan(clamped_theta);
}

bool FrenetHybridAStarPlanner::headingTransitionFeasible(
  double d_start,
  double d_end,
  double theta_start,
  double theta_end) const
{
  if (config_.max_heading_mismatch_deg <= 0.0) {
    return true;
  }
  const double max_mismatch = degreesToRadians(config_.max_heading_mismatch_deg);
  const double avg_theta = std::atan((d_end - d_start) / config_.layer_spacing_m);
  return std::abs(normalizeHeadingError(avg_theta - theta_start)) <= max_mismatch &&
         std::abs(normalizeHeadingError(avg_theta - theta_end)) <= max_mismatch;
}

bool FrenetHybridAStarPlanner::isBetterPartial(
  const SearchState & candidate,
  const SearchState & incumbent) const
{
  return candidate.reached &&
         (candidate.layer_idx > incumbent.layer_idx ||
         (candidate.layer_idx == incumbent.layer_idx &&
         candidate.g_cost < incumbent.g_cost - kTieEpsilon));
}

double FrenetHybridAStarPlanner::heuristicShotTime(
  int layer_idx,
  double s_start,
  double d,
  double theta_rel,
  LocalPlannerIntent intent,
  int final_layer) const
{
  const double remaining_s = static_cast<double>(final_layer - layer_idx) * config_.layer_spacing_m;
  if (remaining_s <= kEpsilon) {
    return 0.0;
  }

  const double slope = headingToSlope(theta_rel);
  const double hold_offset = estimateQuinticShotTime(s_start, remaining_s, d, slope, d, 0.0);
  const double straight = estimateStraightShotTime(s_start, remaining_s, d, slope);

  switch (intent) {
    case LocalPlannerIntent::OVERTAKE:
      return std::min(hold_offset, straight);
    case LocalPlannerIntent::MERGE:
      return estimateQuinticShotTime(s_start, remaining_s, d, slope, 0.0, 0.0);
    case LocalPlannerIntent::FOLLOW_RACING_LINE:
    default:
      return std::min(
        estimateQuinticShotTime(s_start, remaining_s, d, slope, 0.0, 0.0),
        std::min(hold_offset, straight));
  }
}

double FrenetHybridAStarPlanner::estimateQuinticShotTime(
  double s_start,
  double remaining_s,
  double d_start,
  double slope_start,
  double d_end,
  double slope_end) const
{
  if (remaining_s <= kEpsilon) {
    return 0.0;
  }

  const QuinticPolynomial curve = computeQuintic(
    d_start, slope_start, d_end, slope_end, remaining_s);
  const int sample_count = std::max(2, config_.heuristic_sample_count);
  const double max_path_angle = degreesToRadians(config_.max_path_angle_deg);
  std::vector<Point> samples;
  samples.reserve(static_cast<size_t>(sample_count));

  for (int i = 0; i < sample_count; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(sample_count - 1);
    const double d = curve.evaluate(t);
    if (std::abs(d) > config_.max_lateral_offset_m + kEpsilon) {
      return kLargeHeuristic;
    }
    const double path_slope = curve.evaluateDerivative(t) / curve.delta_s;
    if (std::abs(std::atan(path_slope)) > max_path_angle) {
      return kLargeHeuristic;
    }
    samples.push_back(frenet_converter_.frenetToCartesian({s_start + t * remaining_s, d}));
  }

  return estimateSampledShotTime(samples, s_start, remaining_s);
}

double FrenetHybridAStarPlanner::estimateStraightShotTime(
  double s_start,
  double remaining_s,
  double d_start,
  double slope_start) const
{
  if (remaining_s <= kEpsilon) {
    return 0.0;
  }

  const int sample_count = std::max(2, config_.heuristic_sample_count);
  std::vector<Point> samples;
  samples.reserve(static_cast<size_t>(sample_count));

  for (int i = 0; i < sample_count; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(sample_count - 1);
    const double ds = t * remaining_s;
    const double d = d_start + slope_start * ds;
    if (std::abs(d) > config_.max_lateral_offset_m + kEpsilon) {
      return kLargeHeuristic;
    }
    samples.push_back(frenet_converter_.frenetToCartesian({s_start + ds, d}));
  }

  return estimateSampledShotTime(samples, s_start, remaining_s);
}

double FrenetHybridAStarPlanner::estimateSampledShotTime(
  const std::vector<Point> & samples,
  double s_start,
  double delta_s) const
{
  if (samples.size() < 2) {
    return 0.0;
  }

  std::vector<double> curvatures(samples.size(), 0.0);
  for (int i = 1; i + 1 < static_cast<int>(samples.size()); ++i) {
    curvatures[static_cast<size_t>(i)] = computeCurvature(
      samples[static_cast<size_t>(i - 1)],
      samples[static_cast<size_t>(i)],
      samples[static_cast<size_t>(i + 1)]);
  }
  if (samples.size() > 2) {
    curvatures.front() = curvatures[1];
    curvatures.back() = curvatures[samples.size() - 2];
  }

  double time = 0.0;
  const double sample_denominator = static_cast<double>(samples.size() - 1);
  for (int i = 1; i < static_cast<int>(samples.size()); ++i) {
    const double t_prev = static_cast<double>(i - 1) / sample_denominator;
    const double t_curr = static_cast<double>(i) / sample_denominator;
    const double v_prev = computeVelocity(
      s_start + t_prev * delta_s,
      curvatures[static_cast<size_t>(i - 1)],
      frenet_converter_,
      config_);
    const double v_curr = computeVelocity(
      s_start + t_curr * delta_s,
      curvatures[static_cast<size_t>(i)],
      frenet_converter_,
      config_);
    const double velocity = std::max(config_.min_velocity_mps, 0.5 * (v_prev + v_curr));
    time +=
      distance(samples[static_cast<size_t>(i - 1)], samples[static_cast<size_t>(i)]) / velocity;
  }

  return time;
}

std::vector<Point> FrenetHybridAStarPlanner::reconstructPath(
  const std::vector<SearchState> & states,
  int selected_index) const
{
  std::vector<std::vector<Point>> segments;
  int state_index = selected_index;
  while (state_index >= 0) {
    const SearchState & state = states[static_cast<size_t>(state_index)];
    if (!state.reached) {
      return {};
    }
    if (state.parent_index < 0) {
      break;
    }
    segments.push_back(state.edge_samples);
    state_index = state.parent_index;
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
