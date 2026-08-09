#include "local_planning/maneuvers/maneuver_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace local_planning
{
namespace
{

constexpr double kTolerance = 1e-6;

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

void requireNonEmpty(const std::vector<double> & values, const char * name)
{
  if (values.empty()) {
    throw std::invalid_argument(std::string(name) + " must not be empty");
  }
}

void requireFinite(const std::vector<double> & values, const char * name)
{
  if (std::any_of(values.begin(), values.end(), [](double value) {return !std::isfinite(value);})) {
    throw std::invalid_argument(std::string(name) + " must contain only finite values");
  }
}

void validateConfig(const ManeuverConfig & config)
{
  if (!std::isfinite(config.horizon_m) || config.horizon_m <= 0.0) {
    throw std::invalid_argument("horizon_m must be finite and positive");
  }

  requireNonEmpty(
    config.overtake_s_offsets_from_opponent_rear_m,
    "overtake_s_offsets_from_opponent_rear_m");
  requireNonEmpty(config.passing_d_magnitudes_m, "passing_d_magnitudes_m");
  requireNonEmpty(config.overtake_heading_offsets_rad, "overtake_heading_offsets_rad");
  requireNonEmpty(config.overtake_curvature_multipliers, "overtake_curvature_multipliers");
  requireNonEmpty(config.pass_transition_distances_m, "pass_transition_distances_m");
  requireNonEmpty(config.merge_completion_distances_m, "merge_completion_distances_m");

  requireFinite(
    config.overtake_s_offsets_from_opponent_rear_m,
    "overtake_s_offsets_from_opponent_rear_m");
  requireFinite(config.passing_d_magnitudes_m, "passing_d_magnitudes_m");
  requireFinite(config.overtake_heading_offsets_rad, "overtake_heading_offsets_rad");
  requireFinite(config.overtake_curvature_multipliers, "overtake_curvature_multipliers");
  requireFinite(config.pass_transition_distances_m, "pass_transition_distances_m");
  requireFinite(config.merge_completion_distances_m, "merge_completion_distances_m");

  if (std::any_of(
      config.passing_d_magnitudes_m.begin(), config.passing_d_magnitudes_m.end(),
      [](double magnitude) {return magnitude <= 0.0;}))
  {
    throw std::invalid_argument("passing_d_magnitudes_m must contain only positive values");
  }
  if (!std::isfinite(config.collision_circle_radius_m) || config.collision_circle_radius_m <= 0.0) {
    throw std::invalid_argument("collision_circle_radius_m must be finite and positive");
  }
  const double side_deadband = config.sideDeadbandM();
  if (std::any_of(
      config.passing_d_magnitudes_m.begin(), config.passing_d_magnitudes_m.end(),
      [side_deadband](double magnitude) {return magnitude <= side_deadband;}))
  {
    throw std::invalid_argument(
      "passing_d_magnitudes_m must be greater than vehicle width (2 * collision radius)");
  }
  if (std::any_of(
      config.overtake_curvature_multipliers.begin(),
      config.overtake_curvature_multipliers.end(),
      [](double multiplier) {return multiplier < 0.0 || multiplier > 1.0;}))
  {
    throw std::invalid_argument("overtake curvature multipliers must be in [0, 1]");
  }

  const auto invalid_distance = [&config](double distance) {
      return distance <= 0.0 || distance > config.horizon_m;
    };
  if (std::any_of(
      config.pass_transition_distances_m.begin(), config.pass_transition_distances_m.end(),
      invalid_distance))
  {
    throw std::invalid_argument("pass transition distances must be in (0, horizon_m]");
  }
  if (std::any_of(
      config.merge_completion_distances_m.begin(), config.merge_completion_distances_m.end(),
      invalid_distance))
  {
    throw std::invalid_argument("merge completion distances must be in (0, horizon_m]");
  }
}

bool matches(const CurveSample & sample, const BoundaryState & boundary)
{
  return std::hypot(sample.x - boundary.x, sample.y - boundary.y) <= kTolerance &&
         std::abs(std::atan2(
      std::sin(sample.heading - boundary.heading), std::cos(sample.heading - boundary.heading))) <=
         kTolerance && std::abs(sample.curvature - boundary.curvature) <= kTolerance;
}

} // namespace

ManeuverBuilder::ManeuverBuilder(
  const RacelineReference & reference,
  const CurveConnectionGenerator & curve_generator,
  ManeuverConfig config)
: reference_(reference), curve_generator_(curve_generator), config_(std::move(config))
{
  validateConfig(config_);
  removeDuplicates(config_.overtake_s_offsets_from_opponent_rear_m);
  removeDuplicates(config_.passing_d_magnitudes_m);
  removeDuplicates(config_.overtake_heading_offsets_rad);
  removeDuplicates(config_.overtake_curvature_multipliers);
  removeDuplicates(config_.pass_transition_distances_m);
  std::sort(
    config_.pass_transition_distances_m.begin(), config_.pass_transition_distances_m.end(),
    std::greater<double>());
  removeDuplicates(config_.merge_completion_distances_m);
}

bool ManeuverBuilder::boundary(
  double s,
  double d,
  double heading_offset,
  BoundaryState & result,
  double curvature_multiplier) const
{
  const ReferenceGeometrySample reference = reference_.sampleAtS(s);
  const double denominator = 1.0 - d * reference.curvature;
  if (denominator <= kTolerance || !std::isfinite(denominator)) {
    return false;
  }
  result = {
    reference.x + d * reference.normal_x,
    reference.y + d * reference.normal_y,
    reference.heading + heading_offset,
    curvature_multiplier * reference.curvature / denominator,
    reference.velocity};
  return std::isfinite(result.x) && std::isfinite(result.y) &&
         std::isfinite(result.heading) && std::isfinite(result.curvature);
}

bool ManeuverBuilder::connect(
  Path & path,
  const BoundaryState & start,
  const BoundaryState & end,
  double start_raceline_s,
  double end_raceline_s,
  BoundaryState * actual_end) const
{
  const GeneratedConnection connection = curve_generator_.generate({start, end});
  if (!connection.valid || connection.samples.empty()) {
    return false;
  }
  const double s_offset = path.empty() ? 0.0 : path.back().s;
  const double connection_length = connection.samples.back().s;
  const double reference_progress = reference_.deltaS(start_raceline_s, end_raceline_s);
  for (std::size_t i = path.empty() ? 0 : 1; i < connection.samples.size(); ++i) {
    CurveSample sample = connection.samples[i];
    const double fraction = connection_length > kTolerance ? sample.s / connection_length : 0.0;
    sample.raceline_s = reference_.wrapS(start_raceline_s + fraction * reference_progress);
    sample.s += s_offset;
    path.push_back(sample);
  }
  if (actual_end != nullptr) {
    *actual_end = connection.actual_terminal;
  }
  return true;
}

bool ManeuverBuilder::appendTail(Path & path, double start_s, double distance, double d) const
{
  if (distance <= 0.0) {
    return true;
  }
  if (path.empty() || curve_generator_.config().sample_spacing_m <= 0.0) {
    return false;
  }
  BoundaryState start;
  if (!boundary(start_s, d, 0.0, start) || !matches(path.back(), start)) {
    return false;
  }
  for (double covered = 0.0; covered < distance; ) {
    covered += std::min(curve_generator_.config().sample_spacing_m, distance - covered);
    BoundaryState next;
    if (!boundary(start_s + covered, d, 0.0, next)) {
      return false;
    }
    const CurveSample & previous = path.back();
    path.push_back({
        previous.s + std::hypot(next.x - previous.x, next.y - previous.y),
        next.x, next.y, next.heading, next.curvature, 0.0,
        reference_.wrapS(start_s + covered)});
  }
  return true;
}

ManeuverBuilder::SideCheck ManeuverBuilder::sideAndDeviation(
  const Path & path,
  int side,
  bool allow_start_center,
  double target_d) const
{
  const double deadband = config_.sideDeadbandM();
  SideCheck result;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const CurveSample & sample = path[i];
    // Every sample already knows its station -- connect() and appendTail() set
    // raceline_s when they build it -- so this needs no search.  The old
    // project() call here rescanned about twenty spline segments per sample to
    // recover a value the sample was carrying, and at two sweeps per candidate
    // that was 99% of the cost of PASS.
    const Point p(sample.x, sample.y);
    bool converged = false;
    double d = reference_.lateralOffsetAt(p, sample.raceline_s, &converged);
    ++station_hint_stats_.samples;
    if (!converged) {
      ++station_hint_stats_.fallbacks;
      // connect() interpolates raceline_s linearly along the curve, which stops
      // resembling the reference when a corner is tight relative to horizon_m.
      // Fall back to the windowed search this used to do unconditionally, seeded
      // on the sample's own station rather than the previous sample's result --
      // strictly the better seed, and no worse than the old behaviour.
      d = reference_.project(p, sample.raceline_s).d;
    }

    const bool skip_side_check = i == 0 && allow_start_center && std::abs(d) <= deadband;
    // Inside ±deadband is still "on the line"; only reject a clear opposite-side
    // excursion beyond one vehicle width.
    if (!skip_side_check && side * d <= -deadband) {
      return result;   // stays_on_side stays false; the deviation is never read
    }
    result.max_offset_deviation_m =
      std::max(result.max_offset_deviation_m, std::abs(d - target_d));
  }
  result.stays_on_side = true;
  return result;
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

double ManeuverBuilder::preferredOffset(double ego_d) const
{
  double preferred = 0.0;
  double nearest = std::numeric_limits<double>::infinity();
  for (double d : offsets(sideOf(ego_d))) {
    const double distance = std::abs(d - ego_d);
    if (distance < nearest - kTolerance ||
      (std::abs(distance - nearest) <= kTolerance && std::abs(d) < std::abs(preferred)))
    {
      preferred = d;
      nearest = distance;
    }
  }
  return preferred;
}

int ManeuverBuilder::sideOf(double d) const
{
  if (std::abs(d) <= config_.sideDeadbandM()) {
    return 0;
  }
  return d > 0.0 ? 1 : -1;
}

std::vector<ManeuverCandidate> ManeuverBuilder::overtake(
  const BoundaryState & ego,
  double ego_s,
  double ego_d,
  double opponent_rear_s) const
{
  std::vector<ManeuverCandidate> candidates;
  if (!reference_.valid()) {
    return candidates;
  }
  // Unused: the dense side check is deferred to opponent prediction (review P0-1).
  // Crossing needs curvature_multiplier == 0.0 on corners tighter than ~3 m; the
  // interim mitigation is dropping 0.0 from overtake_curvature_multipliers.
  (void)ego_d;
  const double horizon_s = reference_.wrapS(ego_s + config_.horizon_m);
  const std::vector<double> lateral_offsets = offsets(0);
  // The first leg is a function of (intermediate_s, intermediate_d,
  // heading_offset, curvature_multiplier).  It does not depend on horizon_d,
  // which the emission order below nests outside it, so building it inline
  // re-solves the same G2 connection once per same-side horizon offset -- two
  // thirds of the first-leg solves here are exact duplicates.  Solve each
  // distinct first leg once per intermediate_d into this scratch table instead
  // and reuse it as the prefix for every horizon offset.  The loop nesting is
  // otherwise unchanged, so candidates come out in the same order as before;
  // selectOvertake() breaks ties on first-seen, and that must not shift.
  struct FirstLeg
  {
    Path path;
    BoundaryState join;
    bool valid = false;
  };
  const std::size_t heading_count = config_.overtake_heading_offsets_rad.size();
  const std::size_t curvature_count = config_.overtake_curvature_multipliers.size();
  std::vector<FirstLeg> first_legs(heading_count * curvature_count);

  for (double s_offset : config_.overtake_s_offsets_from_opponent_rear_m) {
    const double intermediate_s = reference_.wrapS(opponent_rear_s + s_offset);
    const double progress = reference_.deltaS(ego_s, intermediate_s);
    if (progress <= 0.0 || progress >= config_.horizon_m) {
      continue;
    }
    for (double intermediate_d : lateral_offsets) {
      for (std::size_t h = 0; h < heading_count; ++h) {
        for (std::size_t c = 0; c < curvature_count; ++c) {
          FirstLeg & leg = first_legs[h * curvature_count + c];
          leg.path.clear();   // clear(), not a fresh Path: the capacity is worth keeping
          leg.valid = false;
          BoundaryState intermediate;
          if (!boundary(
              intermediate_s, intermediate_d, config_.overtake_heading_offsets_rad[h],
              intermediate, config_.overtake_curvature_multipliers[c]))
          {
            continue;
          }
          leg.valid = connect(leg.path, ego, intermediate, ego_s, intermediate_s, &leg.join);
        }
      }

      for (double horizon_d : lateral_offsets) {
        if (intermediate_d * horizon_d <= 0.0) {
          continue;
        }
        // Invariant across the heading/curvature pairs below, unlike the first leg.
        BoundaryState horizon;
        if (!boundary(horizon_s, horizon_d, 0.0, horizon)) {
          continue;
        }
        for (std::size_t h = 0; h < heading_count; ++h) {
          for (std::size_t c = 0; c < curvature_count; ++c) {
            const FirstLeg & leg = first_legs[h * curvature_count + c];
            if (!leg.valid) {
              continue;
            }
            Path path = leg.path;
            if (connect(path, leg.join, horizon, intermediate_s, horizon_s)) {
              candidates.push_back({std::move(path), horizon_d, config_.horizon_m, 0.0});
            }
          }
        }
      }
    }
  }
  return candidates;
}

std::vector<ManeuverCandidate> ManeuverBuilder::pass(
  const BoundaryState & ego,
  double ego_s,
  double ego_d) const
{
  std::vector<ManeuverCandidate> candidates;
  const int side = sideOf(ego_d);
  if (!reference_.valid() || side == 0) {
    return candidates;
  }
  const double target_s = reference_.wrapS(ego_s + config_.horizon_m);
  const double target_d = preferredOffset(ego_d);
  BoundaryState target;
  Path path;
  if (boundary(target_s, target_d, 0.0, target) &&
    connect(path, ego, target, ego_s, target_s))
  {
    const SideCheck check = sideAndDeviation(path, side, false, target_d);
    if (check.stays_on_side) {
      candidates.push_back(
        {std::move(path), target_d, config_.horizon_m, check.max_offset_deviation_m});
    }
  }
  return candidates;
}

std::vector<ManeuverCandidate> ManeuverBuilder::recover(
  const BoundaryState & ego,
  double ego_s,
  double ego_d) const
{
  std::vector<ManeuverCandidate> candidates;
  const int side = sideOf(ego_d);
  if (!reference_.valid() || side == 0) {
    return candidates;
  }
  const double preferred = preferredOffset(ego_d);
  for (double transition : config_.pass_transition_distances_m) {
    for (double d : offsets(side)) {
      if (std::abs(d - preferred) <= kTolerance) {
        continue;
      }
      const double target_s = reference_.wrapS(ego_s + transition);
      BoundaryState target;
      Path path;
      if (boundary(target_s, d, 0.0, target) &&
        connect(path, ego, target, ego_s, target_s) &&
        appendTail(path, target_s, config_.horizon_m - transition, d))
      {
        // Deviation is measured against the preferred offset, not this
        // candidate's own d: recovery candidates are ranked by how far they
        // stray from where the planner would rather be.
        const SideCheck check = sideAndDeviation(path, side, false, preferred);
        if (check.stays_on_side) {
          candidates.push_back(
            {std::move(path), d, transition, check.max_offset_deviation_m});
        }
      }
    }
  }
  return candidates;
}

std::vector<ManeuverCandidate> ManeuverBuilder::merge(
  const BoundaryState & ego,
  double ego_s) const
{
  std::vector<ManeuverCandidate> candidates;
  if (!reference_.valid()) {
    return candidates;
  }
  for (double completion : config_.merge_completion_distances_m) {
    const double target_s = reference_.wrapS(ego_s + completion);
    BoundaryState target;
    Path path;
    if (boundary(target_s, 0.0, 0.0, target) &&
      connect(path, ego, target, ego_s, target_s) &&
      appendTail(path, target_s, config_.horizon_m - completion, 0.0))
    {
      candidates.push_back({std::move(path), 0.0, completion, 0.0});
    }
  }
  return candidates;
}

} // namespace local_planning
