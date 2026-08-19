#include "local_planning/maneuvers/maneuver_builder.hpp"

#include "local_planning/core/geometry.hpp"
#include "local_planning/curves/frenet_polynomial.hpp"
#include "worker_pool.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace local_planning
{
namespace
{

constexpr double kTolerance = kGridEps;
// Past this the heading error is effectively perpendicular to the reference and
// tan() stops being a usable encoding of it.  The generator's max_path_angle_deg
// rejects long before here; this only keeps the arithmetic finite.
constexpr double kMaxStartHeadingErrorRad = 1.5;

void removeDuplicates(std::vector<double> & values)
{
  std::vector<double> unique;
  for (double value : values) {
    const bool already_present = std::any_of(
      unique.begin(), unique.end(),
      [value](double existing) {return std::abs(existing - value) <= kTolerance;});
    if (!already_present) {
      unique.push_back(value);
    }
  }
  values = std::move(unique);
}

} // namespace

ManeuverBuilder::ManeuverBuilder(
  const RacelineReference & reference,
  const FrenetConnectionGenerator & curve_generator,
  ManeuverConfig config,
  VehicleGeometry vehicle_geometry)
: reference_(reference), curve_generator_(curve_generator), config_(std::move(config)),
  vehicle_geometry_(vehicle_geometry)
{
  removeDuplicates(config_.overtake_s_offsets_from_opponent_rear_m);
  removeDuplicates(config_.passing_d_magnitudes_m);
  removeDuplicates(config_.overtake_heading_offsets_rad);
  removeDuplicates(config_.pass_transition_distances_m);
  std::sort(
    config_.pass_transition_distances_m.begin(), config_.pass_transition_distances_m.end(),
    std::greater<double>());
  removeDuplicates(config_.merge_completion_distances_m);

  const bool has_zero_heading = std::any_of(
    config_.overtake_heading_offsets_rad.begin(),
    config_.overtake_heading_offsets_rad.end(),
    [](double heading) {return std::abs(heading) <= kTolerance;});
  if (!has_zero_heading) {
    throw std::invalid_argument("overtake_heading_offsets_rad must include 0.0");
  }
}

bool ManeuverBuilder::prepareWindow(const BoundaryState & ego, double ego_s) const
{
  plan_speed_mps_ = ego.speed;
  return window_.build(
    reference_, ego_s, config_.horizon_m, curve_generator_.config().sample_spacing_m);
}

bool ManeuverBuilder::startBoundary(
  const BoundaryState & ego,
  double ego_s,
  double ego_d,
  FrenetBoundary & result) const
{
  (void)ego_s;   // index 0 of the window is exactly ego_s by construction
  if (!window_.valid()) {
    return false;
  }
  const ReferenceGeometrySample & reference = window_.at(0);
  const double tangent_scale = 1.0 - ego_d * reference.curvature;
  if (!(tangent_scale > kTolerance) || !std::isfinite(tangent_scale)) {
    return false;
  }
  const double heading_error = shortestAngleDiff(ego.heading, reference.heading);
  if (!std::isfinite(heading_error) || std::abs(heading_error) >= kMaxStartHeadingErrorRad) {
    return false;
  }

  result.s = window_.startS();
  result.d = ego_d;
  // heading = ref.heading + atan2(d', A), inverted.
  result.d_prime = tangent_scale * std::tan(heading_error);
  result.d_double_prime = frenetSecondDerivativeForVehicleCurvature(
    ego.curvature, ego_d, result.d_prime, reference.curvature,
    reference.curvature_derivative);
  return std::isfinite(result.d_prime) && std::isfinite(result.d_double_prime);
}

bool ManeuverBuilder::boundary(
  double s,
  double d,
  double heading_offset,
  FrenetBoundary & result) const
{
  if (!window_.valid()) {
    return false;
  }
  const std::size_t index = window_.indexForS(s);
  const ReferenceGeometrySample & reference = window_.at(index);
  const double tangent_scale = 1.0 - d * reference.curvature;
  if (!(tangent_scale > kTolerance) || !std::isfinite(tangent_scale)) {
    return false;
  }
  result.s = window_.sAt(index);
  result.d = d;
  result.d_prime = tangent_scale * std::tan(heading_offset);
  result.d_double_prime = 0.0;
  return std::isfinite(result.d_prime);
}

bool ManeuverBuilder::connect(
  Path & path,
  const FrenetBoundary & start,
  const FrenetBoundary & end,
  double & max_abs_d) const
{
  const std::size_t i_start = window_.indexForS(start.s);
  const std::size_t i_end = window_.indexForS(end.s);
  if (i_end <= i_start) {
    return false;
  }
  const double delta_s =
    window_.spacingM() * static_cast<double>(i_end - i_start);
  const FrenetPolynomial polynomial = computeQuintic(
    start.d, start.d_prime, start.d_double_prime,
    end.d, end.d_prime, end.d_double_prime, delta_s);
  const FrenetConnectionResult result =
    curve_generator_.generate(window_, i_start, i_end, polynomial, path, plan_speed_mps_);
  if (!result.valid) {
    return false;
  }
  max_abs_d = std::max(max_abs_d, result.max_abs_d);
  return true;
}

bool ManeuverBuilder::appendOffsetTail(
  Path & path,
  double start_s,
  double reference_distance_m,
  double d,
  double & max_abs_d) const
{
  if (path.empty()) {
    return false;
  }
  // A full-horizon transition leaves no tail to append.  That is success with
  // nothing to do, not a failure: the path already ends where the tail would
  // have started.  (MERGE's longest completion distance lands here; PASS
  // recovery now skips its full-horizon entry outright, since pass() covers it.)
  if (reference_distance_m <= kTolerance) {
    return true;
  }
  // A constant offset is just a connection whose two boundaries agree, so this
  // is the same sampler and the same code path as every other leg.  The join is
  // C2 by construction: the leg that ended here ended with d' = d'' = 0 at this
  // same d, which is precisely what these boundaries request.  That is what
  // makes the old matches() continuity gate unnecessary rather than merely
  // loose.
  const FrenetBoundary start{start_s, d, 0.0, 0.0};
  const FrenetBoundary end{start_s + reference_distance_m, d, 0.0, 0.0};
  return connect(path, start, end, max_abs_d);
}

bool ManeuverBuilder::staysOnSide(const Path & path, int side) const
{
  const double deadband = vehicle_geometry_.fullWidthM();
  for (const CurveSample & sample : path) {
    // sample.d is exact and free: it is the quantity the curve was planned in.
    // This used to be a Newton refinement per sample with a windowed-search
    // fallback, which at two sweeps per candidate was the dominant cost of
    // PASS.
    //
    // Inside ±deadband is still "on the line"; only reject a clear
    // opposite-side excursion beyond one vehicle width.
    if (side * sample.d <= -deadband) {
      return false;
    }
  }
  return true;
}

std::vector<double> ManeuverBuilder::offsets(int side) const
{
  std::vector<double> result;
  for (int sign : {-1, 1}) {
    if (side != 0 && sign != side) {
      continue;
    }
    for (double magnitude : config_.passing_d_magnitudes_m) {
      result.push_back(sign * magnitude);
    }
  }
  return result;
}

int ManeuverBuilder::sideOf(double d) const
{
  if (std::abs(d) <= vehicle_geometry_.fullWidthM()) {
    return 0;
  }
  return d > 0.0 ? 1 : -1;
}

bool ManeuverBuilder::beginGeneration(
  const BoundaryState & ego, double ego_s, double ego_d, FrenetBoundary & start) const
{
  return reference_.valid() && prepareWindow(ego, ego_s) &&
         startBoundary(ego, ego_s, ego_d, start);
}

std::vector<ManeuverCandidate> ManeuverBuilder::sampleChains(
  const FrenetBoundary & start, const std::vector<ChainSpec> & chains) const
{
  std::vector<Waypoint> prefixes;
  std::vector<std::size_t> chain_prefix(chains.size(), 0);
  prefixes.reserve(chains.size());
  const auto sameWaypoint = [](const Waypoint & a, const Waypoint & b) {
      return std::abs(a.s - b.s) <= kTolerance &&
             std::abs(a.d - b.d) <= kTolerance &&
             std::abs(a.heading_offset - b.heading_offset) <= kTolerance;
    };
  for (std::size_t i = 0; i < chains.size(); ++i) {
    const std::size_t found = [&]() {
        const Waypoint & first = chains[i].waypoints.front();
        for (std::size_t p = 0; p < prefixes.size(); ++p) {
          if (sameWaypoint(prefixes[p], first)) {
            return p;
          }
        }
        prefixes.push_back(first);
        return prefixes.size() - 1;
      }();
    chain_prefix[i] = found;
  }

  struct Prefix
  {
    Path path;
    FrenetBoundary join;
    double max_abs_d = 0.0;
    bool valid = false;
  };
  std::vector<Prefix> sampled(prefixes.size());
  parallelFor(prefixes.size(), [&](std::size_t i) {
      FrenetBoundary end;
      if (!boundary(prefixes[i].s, prefixes[i].d, prefixes[i].heading_offset, end)) {
        return;
      }
      sampled[i].join = end;
      sampled[i].valid = connect(sampled[i].path, start, end, sampled[i].max_abs_d);
    });

  std::vector<ManeuverCandidate> slots(chains.size());
  std::vector<char> slot_ok(chains.size(), 0);
  parallelFor(chains.size(), [&](std::size_t i) {
      const ChainSpec & chain = chains[i];
      const Prefix & prefix = sampled[chain_prefix[i]];
      if (!prefix.valid) {
        return;
      }
      Path path = prefix.path;
      double max_abs_d = prefix.max_abs_d;
      FrenetBoundary join = prefix.join;
      for (std::size_t w = 1; w < chain.waypoints.size(); ++w) {
        const Waypoint & waypoint = chain.waypoints[w];
        FrenetBoundary end;
        if (!boundary(waypoint.s, waypoint.d, waypoint.heading_offset, end) ||
          !connect(path, join, end, max_abs_d))
        {
          return;
        }
        join = end;
      }
      if (chain.append_tail &&
        !appendOffsetTail(
          path, chain.tail_start_s, chain.tail_distance_m, chain.tail_d, max_abs_d))
      {
        return;
      }
      if (chain.side != 0 && !staysOnSide(path, chain.side)) {
        return;
      }
      slots[i] = {
        std::move(path), chain.passing_d, chain.terminal_d, chain.maneuver_distance_m,
        max_abs_d, chain.uses_offset_tail};
      slot_ok[i] = 1;
    });

  std::vector<ManeuverCandidate> candidates;
  candidates.reserve(slots.size());
  for (std::size_t i = 0; i < slots.size(); ++i) {
    if (slot_ok[i]) {
      candidates.push_back(std::move(slots[i]));
    }
  }
  return candidates;
}

std::vector<ManeuverCandidate> ManeuverBuilder::overtake(
  const BoundaryState & ego,
  double ego_s,
  double ego_d,
  double opponent_rear_s) const
{
  FrenetBoundary start;
  if (!beginGeneration(ego, ego_s, ego_d, start)) {
    return {};
  }
  const double horizon_s = reference_.wrapS(ego_s + config_.horizon_m);
  const std::vector<double> lateral_offsets = offsets(0);
  std::vector<ChainSpec> chains;
  for (double s_offset : config_.overtake_s_offsets_from_opponent_rear_m) {
    const double intermediate_s = reference_.wrapS(opponent_rear_s + s_offset);
    const double progress = reference_.deltaS(ego_s, intermediate_s);
    if (!(progress > 0.0 && progress < config_.horizon_m)) {
      continue;
    }
    for (double passing_d : lateral_offsets) {
      for (double terminal_d : lateral_offsets) {
        if (passing_d * terminal_d <= 0.0) {
          continue;
        }
        for (double heading : config_.overtake_heading_offsets_rad) {
          chains.push_back({
              {{intermediate_s, passing_d, heading}, {horizon_s, terminal_d, 0.0}},
              passing_d, terminal_d, config_.horizon_m, false});
        }
      }
      chains.push_back({
          {{intermediate_s, passing_d, 0.0}},
          passing_d, passing_d, config_.horizon_m, true, 0, true,
          intermediate_s, config_.horizon_m - progress, passing_d});
    }
  }
  return sampleChains(start, chains);
}

std::vector<ManeuverCandidate> ManeuverBuilder::pass(
  const BoundaryState & ego, double ego_s, double ego_d) const
{
  const int side = sideOf(ego_d);
  if (side == 0) {
    return {};
  }
  FrenetBoundary start;
  if (!beginGeneration(ego, ego_s, ego_d, start)) {
    return {};
  }
  const double target_s = reference_.wrapS(ego_s + config_.horizon_m);
  std::vector<ChainSpec> chains;
  for (double target_d : offsets(side)) {
    chains.push_back({
        {{target_s, target_d, 0.0}},
        target_d, target_d, config_.horizon_m, false, side});
  }
  return sampleChains(start, chains);
}

std::vector<ManeuverCandidate> ManeuverBuilder::recover(
  const BoundaryState & ego, double ego_s, double ego_d) const
{
  const int side = sideOf(ego_d);
  if (side == 0) {
    return {};
  }
  FrenetBoundary start;
  if (!beginGeneration(ego, ego_s, ego_d, start)) {
    return {};
  }
  std::vector<ChainSpec> chains;
  for (double transition : config_.pass_transition_distances_m) {
    if (std::abs(transition - config_.horizon_m) <= kTolerance) {
      continue;
    }
    for (double d : offsets(side)) {
      const double target_s = reference_.wrapS(ego_s + transition);
      chains.push_back({
          {{target_s, d, 0.0}},
          d, d, transition, transition < config_.horizon_m - kTolerance, side, true,
          target_s, config_.horizon_m - transition, d});
    }
  }
  return sampleChains(start, chains);
}

std::vector<ManeuverCandidate> ManeuverBuilder::merge(
  const BoundaryState & ego, double ego_s, double ego_d) const
{
  FrenetBoundary start;
  if (!beginGeneration(ego, ego_s, ego_d, start)) {
    return {};
  }
  std::vector<ChainSpec> chains;
  for (double completion : config_.merge_completion_distances_m) {
    const double target_s = reference_.wrapS(ego_s + completion);
    chains.push_back({
        {{target_s, 0.0, 0.0}},
        0.0, 0.0, completion, false, 0, true,
        target_s, config_.horizon_m - completion, 0.0});
  }
  return sampleChains(start, chains);
}

} // namespace local_planning
