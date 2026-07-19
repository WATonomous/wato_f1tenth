#include "planning/planner/local_frenet_lattice_planner.hpp"

#include "planning/planner/collision_checker.hpp"
#include "planning/planner/edge_evaluator.hpp"
#include "planning/planner/frenet_polynomial.hpp"
#include "planning/planner/path_processing.hpp"
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
constexpr double kPi = 3.14159265358979323846;

double frenetSecondDerivativeForVehicleCurvature(
  double vehicle_curvature,
  double lateral_offset,
  double lateral_slope,
  double reference_curvature,
  double reference_curvature_derivative)
{
  // For x(s) = r(s) + d(s)n(s), the Cartesian path curvature is:
  // [k_ref A^2 + A d'' + k_ref' d d' + 2 k_ref d'^2] /
  // (A^2 + d'^2)^(3/2), where A = 1 - k_ref d.
  // Solve this expression for d'' so the first lattice edge continues the
  // curvature implied by the current steering angle.
  const double tangent_scale = 1.0 - reference_curvature * lateral_offset;
  if (std::abs(tangent_scale) <= kEpsilon) {
    // The Frenet chart is singular here.  Preserve the previous small-angle
    // conversion instead of amplifying numerical error.
    return vehicle_curvature - reference_curvature;
  }

  const double tangent_norm_squared =
    tangent_scale * tangent_scale + lateral_slope * lateral_slope;
  const double tangent_norm_cubed =
    tangent_norm_squared * std::sqrt(tangent_norm_squared);
  return (
    vehicle_curvature * tangent_norm_cubed -
    reference_curvature * tangent_scale * tangent_scale -
    reference_curvature_derivative * lateral_offset * lateral_slope -
    2.0 * reference_curvature * lateral_slope * lateral_slope) /
    tangent_scale;
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
  const Odometry & odom,
  const OccupancyGrid & grid,
  LocalPlannerIntent intent,
  std::chrono::steady_clock::time_point deadline)
{
  LocalFrenetPlan result;

  if (frenet_converter_.getTotalLength() <= kEpsilon ||
    config_.layer_spacing_m <= kEpsilon ||
    config_.lane_spacing_m <= kEpsilon ||
    config_.sample_spacing_m <= kEpsilon ||
    config_.max_path_angle_deg <= 0.0 ||
    config_.max_path_angle_deg >= 90.0)
  {
    result.status = LocalFrenetPlan::Status::INVALID_REFERENCE;
    return result;
  }
  FrenetPoint start = frenet_converter_.cartesianToFrenet(odom.position);
  const std::vector<double> lanes = generateLaneOffsets();
  const int layer_count =
    std::max(1, static_cast<int>(std::ceil(config_.horizon_m / config_.layer_spacing_m)));
  const int sample_count =
    std::max(
    2, static_cast<int>(std::ceil(
      config_.layer_spacing_m / config_.sample_spacing_m)) + 1);
  const int lane_count = static_cast<int>(lanes.size());
  const ReferenceGeometrySample start_ref = frenet_converter_.sampleAtS(start.s);
  const double heading_error = normalizeHeadingError(odom.heading - start_ref.heading);
  const double start_slope = std::clamp(std::tan(heading_error), -1.5, 1.5);
  start.slope = start_slope;
  if (odom.has_steering_angle && config_.wheelbase_m > kEpsilon) {
    const double vehicle_curvature = std::tan(odom.steering_angle) / config_.wheelbase_m;
    start.second_derivative = frenetSecondDerivativeForVehicleCurvature(
      vehicle_curvature,
      start.d,
      start.slope,
      start_ref.curvature,
      frenet_converter_.getRacingLineCurvatureDerivative(start.s));
  }

  frenet_converter_.fillUniformReferenceGeometryTable(
    start.s, layer_count, config_.layer_spacing_m, sample_count, reference_geometry_table_);

  std::vector<std::vector<DpState>> states(
    static_cast<size_t>(layer_count + 1),
    std::vector<DpState>(static_cast<size_t>(lane_count)));

  CollisionChecker collision_checker(config_);
  FrenetEdgeEvaluator edge_evaluator(config_, collision_checker);
  EdgeEvaluationScratch edge_scratch;

  const double clearance_cap_m = std::max(0.0, config_.soft_inflation_distance_m);
  const double max_slope = std::tan(config_.max_path_angle_deg * kPi / 180.0);

  /*
  the measured car state is a special DP source: try a direct quartic from the
  car to every lane in every forward layer.  each valid quartic is an additive
  cost-to-come competitor at its destination node; the ordinary layer-by-layer
  DP below still runs in full and can beat it.  this is extra maneuverability
  work (it is what makes sharp-turn starts reachable), not an optimization.
  */
  const auto direct_phase_start = std::chrono::steady_clock::now();
  result.direct_quartic_diagnostics.assign(
    static_cast<size_t>(layer_count), EdgeLayerDiagnostics{});
  std::vector<ReferenceGeometrySample> direct_span;
  for (int destination_layer = 1; destination_layer <= layer_count; ++destination_layer) {
    EdgeLayerDiagnostics & diagnostics =
      result.direct_quartic_diagnostics[static_cast<size_t>(destination_layer - 1)];
    diagnostics.destination_layer = destination_layer;

    const int span_count = assembleDirectSpan(destination_layer, sample_count, direct_span);
    if (span_count < 2) {
      continue;
    }
    const double span_delta_s =
      direct_span[static_cast<size_t>(span_count - 1)].s - direct_span[0].s;
    if (span_delta_s <= kEpsilon) {
      continue;
    }

    for (int to_lane = 0; to_lane < lane_count; ++to_lane) {
      const double d_end = lanes[static_cast<size_t>(to_lane)];
      if (std::abs(d_end - start.d) / span_delta_s > max_slope) {
        ++diagnostics.angle_pruned;
        continue;
      }

      const FrenetPolynomial curve = computeQuartic(
        start.d, start_slope, start.second_derivative, d_end, 0.0, span_delta_s);
      const EdgeEvaluation edge = edge_evaluator.evaluateEdge(
        curve, intent, grid, direct_span.data(), span_count, edge_scratch);
      if (edge.collision_status == CollisionStatus::GEOMETRY_CONSTRAINT) {
        ++diagnostics.geometry_rejected;
        continue;
      }
      if (edge.collision_status == CollisionStatus::COLLISION) {
        ++diagnostics.collided;
        continue;
      }
      if (edge.collision_status == CollisionStatus::OUT_OF_GRID) {
        ++diagnostics.out_of_grid;
        continue;
      }
      ++diagnostics.accepted;

      // the evaluator averages intent bias over one edge; scale by the number
      // of layer hops spanned so skipping layers cannot make a quartic cheaper
      // than the per-edge sums it competes with
      const double seed_cost = edge.predicted_time_cost +
        edge.intent_bias_cost * static_cast<double>(destination_layer);
      relaxDpNode(
        states[static_cast<size_t>(destination_layer)][static_cast<size_t>(to_lane)],
        seed_cost, edge.predicted_time_cost, edge.minimum_clearance_m,
        clearance_cap_m, -1, true);
    }
  }
  result.direct_quartic_runtime_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - direct_phase_start).count();

  /*
  i could defo make this clearer and use better practice
  its just harder for me to see on my screen if i nest a bunch of if's and get rid of the continues
  TODO: do that^

  nodes: layer (s), lane (d)
  edges: only forward one layer with cubics connecting each hop (layer 1 is
  seeded by the direct quartics above, so there is no layer-0 expansion)
  update: just relax min cost like djikstra's normally does
  layers give a fixed order so all we need is one forward pass
  time complexity is O(layers * lanes * samples * inflation_cells^2) i think
  */
  result.cubic_edge_diagnostics.assign(
    static_cast<size_t>(layer_count), EdgeLayerDiagnostics{});
  for (int layer = 0; layer < layer_count; ++layer) {
    result.cubic_edge_diagnostics[static_cast<size_t>(layer)].destination_layer = layer + 1;
  }
  for (int layer = 1; layer < layer_count; ++layer) {
    const int next_layer = layer + 1;
    EdgeLayerDiagnostics & diagnostics =
      result.cubic_edge_diagnostics[static_cast<size_t>(next_layer - 1)];
    const ReferenceGeometrySample * layer_ref =
      reference_geometry_table_.data() +
      static_cast<std::size_t>(layer) * static_cast<std::size_t>(sample_count);
    const double layer_delta_s = layer_ref[sample_count - 1].s - layer_ref[0].s;
    if (layer_delta_s <= kEpsilon) {
      continue;
    }

    for (int from_lane = 0; from_lane < lane_count; ++from_lane) {
      const DpState & from_state =
        states[static_cast<size_t>(layer)][static_cast<size_t>(from_lane)];
      if (!from_state.reachable) {
        continue;
      }

      const double d0 = lanes[static_cast<size_t>(from_lane)];
      for (int to_lane = 0; to_lane < lane_count; ++to_lane) {
        const double d_end = lanes[static_cast<size_t>(to_lane)];
        if (std::abs(d_end - d0) / config_.layer_spacing_m > max_slope) {
          ++diagnostics.angle_pruned;
          continue;
        }

        const FrenetPolynomial curve = computeCubic(d0, 0.0, d_end, 0.0, layer_delta_s);
        const EdgeEvaluation edge = edge_evaluator.evaluateEdge(
          curve, intent, grid, layer_ref, sample_count, edge_scratch);

        if (edge.collision_status == CollisionStatus::GEOMETRY_CONSTRAINT) {
          ++diagnostics.geometry_rejected;
          continue;
        }
        if (edge.collision_status == CollisionStatus::COLLISION) {
          ++diagnostics.collided;
          continue;
        }
        if (edge.collision_status == CollisionStatus::OUT_OF_GRID) {
          ++diagnostics.out_of_grid;
          continue;
        }
        ++diagnostics.accepted;

        relaxDpNode(
          states[static_cast<size_t>(next_layer)][static_cast<size_t>(to_lane)],
          from_state.seed_cost + edge.total_cost,
          from_state.predicted_time + edge.predicted_time_cost,
          std::min(from_state.minimum_clearance_m, edge.minimum_clearance_m),
          clearance_cap_m, from_lane, false);
      }
    }
  }

  result.reachable_lanes_by_layer.assign(static_cast<size_t>(layer_count), 0);
  for (int layer = 1; layer <= layer_count; ++layer) {
    int reachable = 0;
    for (int lane = 0; lane < lane_count; ++lane) {
      if (states[static_cast<size_t>(layer)][static_cast<size_t>(lane)].reachable) {
        ++reachable;
      }
    }
    result.reachable_lanes_by_layer[static_cast<size_t>(layer - 1)] = reachable;
  }
  if (std::chrono::steady_clock::now() >= deadline) {
    result.status = LocalFrenetPlan::Status::DEADLINE_EXCEEDED;
    return result;
  }
  //at this point the dp table is actually filled
  // Clearance inside the soft threshold dominates. Once every option being
  // compared reaches that threshold, choose the lowest predicted travel time.
  // Partial-horizon fallback: prefer the final layer, but when it has no
  // reachable lane (corridor wall-blocked, off-grid, folded, or pruned) select
  // from the deepest layer that does instead of returning NO_PATH.  A short
  // valid path that steers back toward the corridor beats no path at all.

  int best_lane = -1;
  int selected_final_layer = -1;
  for (int layer = layer_count; layer >= 1 && best_lane < 0; --layer) {
    double best_clearance_rank = -std::numeric_limits<double>::infinity();
    double best_seed_cost = std::numeric_limits<double>::infinity();
    for (int lane = 0; lane < lane_count; ++lane) {
      const DpState & state = states[static_cast<size_t>(layer)][static_cast<size_t>(lane)];
      if (!state.reachable) {
        continue;
      }

      const double clearance_rank = std::min(state.minimum_clearance_m, clearance_cap_m);
      const bool better = clearance_rank > best_clearance_rank ||
        (clearance_rank == best_clearance_rank && state.seed_cost < best_seed_cost) ||
        (clearance_rank == best_clearance_rank &&
        state.seed_cost == best_seed_cost && lane < best_lane);

      if (better) {
        best_lane = lane;
        best_clearance_rank = clearance_rank;
        best_seed_cost = state.seed_cost;
        selected_final_layer = layer;
      }
    }
  }

  if (best_lane < 0) {
    result.status = LocalFrenetPlan::Status::NO_PATH;
    return result;
  }
  result.selected_final_layer = selected_final_layer;

  const SelectedLatticePath selected_path = reconstructSelectedPath(
    states, best_lane, selected_final_layer, lanes, start, intent, grid, edge_evaluator,
    sample_count);
  result.path = selected_path.path;
  result.direct_entry_layer = selected_path.direct_entry_layer;

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
  result.status = result.path.empty() ? LocalFrenetPlan::Status::NO_PATH :
    LocalFrenetPlan::Status::SUCCESS;
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

void LocalFrenetLatticePlanner::relaxDpNode(
  DpState & to_state,
  double new_seed_cost,
  double new_predicted_time,
  double new_minimum_clearance_m,
  double clearance_cap_m,
  int parent_lane,
  bool direct_from_start)
{
  const double new_clearance_rank = std::min(new_minimum_clearance_m, clearance_cap_m);
  const double old_clearance_rank =
    std::min(to_state.minimum_clearance_m, clearance_cap_m);
  const bool improves = !to_state.reachable ||
    new_clearance_rank > old_clearance_rank ||
    (new_clearance_rank == old_clearance_rank && new_seed_cost < to_state.seed_cost);
  if (!improves) {
    return;
  }

  to_state.reachable = true;
  to_state.seed_cost = new_seed_cost;
  to_state.predicted_time = new_predicted_time;
  to_state.minimum_clearance_m = new_minimum_clearance_m;
  to_state.parent_lane = parent_lane;
  to_state.direct_from_start = direct_from_start;
}

int LocalFrenetLatticePlanner::assembleDirectSpan(
  int destination_layer,
  int sample_count,
  std::vector<ReferenceGeometrySample> & span) const
{
  span.clear();
  if (destination_layer < 1 || sample_count < 2) {
    return 0;
  }
  const std::size_t needed =
    static_cast<std::size_t>(destination_layer) * static_cast<std::size_t>(sample_count);
  if (needed > reference_geometry_table_.size()) {
    return 0;
  }

  span.reserve(
    static_cast<std::size_t>(destination_layer) *
    static_cast<std::size_t>(sample_count - 1) + 1);
  for (int layer = 0; layer < destination_layer; ++layer) {
    const ReferenceGeometrySample * block =
      reference_geometry_table_.data() +
      static_cast<std::size_t>(layer) * static_cast<std::size_t>(sample_count);
    for (int i = (layer == 0) ? 0 : 1; i < sample_count; ++i) {
      span.push_back(block[i]);
    }
  }
  return static_cast<int>(span.size());
}

/*
a path is one direct quartic from the car into its entry layer, then ordinary
cubic parent edges layer by layer to the final lane.  the quartic may enter at
any forward layer, including the final one.  final_layer may be shallower than
the lattice horizon when the partial-horizon fallback selected it.
*/
LocalFrenetLatticePlanner::SelectedLatticePath
LocalFrenetLatticePlanner::reconstructSelectedPath(
  const std::vector<std::vector<DpState>> & states,
  int final_lane,
  int final_layer,
  const std::vector<double> & lanes,
  const FrenetPoint & start,
  LocalPlannerIntent intent,
  const OccupancyGrid & grid,
  const FrenetEdgeEvaluator & edge_evaluator,
  int sample_count) const
{
  SelectedLatticePath selected_path;
  if (states.size() < 2 || final_lane < 0 || sample_count < 2 ||
    final_layer < 1 || final_layer >= static_cast<int>(states.size()))
  {
    return selected_path;
  }

  std::vector<int> lane_by_layer(states.size(), -1);
  int lane = final_lane;
  int entry_layer = -1;
  for (int layer = final_layer; layer > 0; --layer) {
    if (lane < 0 || lane >= static_cast<int>(lanes.size())) {
      return {};
    }

    const DpState & state = states[static_cast<size_t>(layer)][static_cast<size_t>(lane)];
    if (!state.reachable) {
      return {};
    }

    lane_by_layer[static_cast<size_t>(layer)] = lane;
    if (state.direct_from_start) {
      entry_layer = layer;
      break;
    }
    lane = state.parent_lane;
    if (lane < 0) {
      return {};
    }
  }
  if (entry_layer < 0) {
    return {};
  }
  selected_path.direct_entry_layer = entry_layer;

  selected_path.anchors.reserve(
    static_cast<std::size_t>(final_layer - entry_layer) + 2);
  selected_path.anchors.push_back(start);
  for (int layer = entry_layer; layer <= final_layer; ++layer) {
    selected_path.anchors.push_back(
      {
        start.s + static_cast<double>(layer) * config_.layer_spacing_m,
        lanes[static_cast<size_t>(lane_by_layer[static_cast<size_t>(layer)])],
        0.0,
        0.0
      });
  }

  EdgeEvaluationScratch edge_scratch;
  std::vector<ReferenceGeometrySample> direct_span;
  const int span_count = assembleDirectSpan(entry_layer, sample_count, direct_span);
  if (span_count < 2) {
    return {};
  }
  const double span_delta_s =
    direct_span[static_cast<size_t>(span_count - 1)].s - direct_span[0].s;
  const FrenetPolynomial direct_curve = computeQuartic(
    start.d, start.slope, start.second_derivative,
    lanes[static_cast<size_t>(lane_by_layer[static_cast<size_t>(entry_layer)])],
    0.0, span_delta_s);
  const EdgeEvaluation direct_edge = edge_evaluator.evaluateEdge(
    direct_curve, intent, grid, direct_span.data(), span_count, edge_scratch);
  if (direct_edge.collision_status != CollisionStatus::FREE) {
    return {};
  }

  selected_path.path = edge_scratch.samples;
  selected_path.path.reserve(
    static_cast<std::size_t>(span_count) +
    static_cast<std::size_t>(final_layer - entry_layer) *
    static_cast<std::size_t>(sample_count - 1));

  for (int layer = entry_layer; layer < final_layer; ++layer) {
    const std::size_t table_offset =
      static_cast<std::size_t>(layer) * static_cast<std::size_t>(sample_count);
    if (table_offset + static_cast<std::size_t>(sample_count) >
      reference_geometry_table_.size())
    {
      return {};
    }
    const ReferenceGeometrySample * layer_ref =
      reference_geometry_table_.data() + table_offset;
    const double layer_delta_s = layer_ref[sample_count - 1].s - layer_ref[0].s;
    const FrenetPolynomial curve = computeCubic(
      lanes[static_cast<size_t>(lane_by_layer[static_cast<size_t>(layer)])],
      0.0,
      lanes[static_cast<size_t>(lane_by_layer[static_cast<size_t>(layer + 1)])],
      0.0,
      layer_delta_s);
    const EdgeEvaluation edge = edge_evaluator.evaluateEdge(
      curve, intent, grid, layer_ref, sample_count, edge_scratch);
    if (edge.collision_status != CollisionStatus::FREE) {
      return {};
    }

    for (size_t i = 1; i < edge_scratch.samples.size(); ++i) {
      selected_path.path.push_back(edge_scratch.samples[i]);
    }
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
