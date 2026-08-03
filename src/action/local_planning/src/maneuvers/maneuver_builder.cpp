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
  BoundaryState * actual_end) const
{
  const GeneratedConnection connection = curve_generator_.generate({start, end});
  if (!connection.valid || connection.samples.empty()) {
    return false;
  }
  const double s_offset = path.empty() ? 0.0 : path.back().s;
  for (std::size_t i = path.empty() ? 0 : 1; i < connection.samples.size(); ++i) {
    CurveSample sample = connection.samples[i];
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
        next.x, next.y, next.heading, next.curvature, 0.0});
  }
  return true;
}

bool ManeuverBuilder::staysOnSide(
  const Path & path,
  double ego_s,
  int side,
  bool allow_start_center) const
{
  const double deadband = config_.sideDeadbandM();
  double seed = reference_.wrapS(ego_s);
  for (std::size_t i = 0; i < path.size(); ++i) {
    const CurveSample & sample = path[i];
    const Projection projection = reference_.project(Point(sample.x, sample.y), seed);
    seed = projection.s;
    if (i == 0 && allow_start_center && std::abs(projection.d) <= deadband) {
      continue;
    }
    // Inside ±deadband is still "on the line"; only reject a clear opposite-side
    // excursion beyond one vehicle width.
    if (side * projection.d <= -deadband) {
      return false;
    }
  }
  return true;
}

double ManeuverBuilder::maximumOffsetDeviation(
  const Path & path,
  double ego_s,
  double target_d) const
{
  double seed = reference_.wrapS(ego_s);
  double maximum_deviation = 0.0;
  for (const CurveSample & sample : path) {
    const Projection projection = reference_.project(Point(sample.x, sample.y), seed);
    seed = projection.s;
    maximum_deviation = std::max(maximum_deviation, std::abs(projection.d - target_d));
  }
  return maximum_deviation;
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
  (void)ego_d;  // kept for the staysOnSide check below when re-enabled
  const double horizon_s = reference_.wrapS(ego_s + config_.horizon_m);
  const std::vector<double> lateral_offsets = offsets(0);
  for (double s_offset : config_.overtake_s_offsets_from_opponent_rear_m) {
    const double intermediate_s = reference_.wrapS(opponent_rear_s + s_offset);
    const double progress = reference_.deltaS(ego_s, intermediate_s);
    if (progress <= 0.0 || progress >= config_.horizon_m) {
      continue;
    }
    for (double intermediate_d : lateral_offsets) {
      for (double horizon_d : lateral_offsets) {
        if (intermediate_d * horizon_d <= 0.0) {
          continue;
        }
        for (double heading_offset : config_.overtake_heading_offsets_rad) {
          for (double curvature_multiplier : config_.overtake_curvature_multipliers) {
            BoundaryState intermediate;
            BoundaryState horizon;
            if (!boundary(
                intermediate_s, intermediate_d, heading_offset, intermediate,
                curvature_multiplier) ||
              !boundary(horizon_s, horizon_d, 0.0, horizon))
            {
              continue;
            }
            Path path;
            BoundaryState join;
            if (connect(path, ego, intermediate, &join) && connect(path, join, horizon))
            {
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
    connect(path, ego, target) && staysOnSide(path, ego_s, side, false))
  {
    const double deviation = maximumOffsetDeviation(path, ego_s, target_d);
    candidates.push_back({std::move(path), target_d, config_.horizon_m, deviation});
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
      if (boundary(target_s, d, 0.0, target) && connect(path, ego, target) &&
        appendTail(path, target_s, config_.horizon_m - transition, d) &&
        staysOnSide(path, ego_s, side, false))
      {
        const double deviation = maximumOffsetDeviation(path, ego_s, preferred);
        candidates.push_back({std::move(path), d, transition, deviation});
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
    if (boundary(target_s, 0.0, 0.0, target) && connect(path, ego, target) &&
      appendTail(path, target_s, config_.horizon_m - completion, 0.0))
    {
      candidates.push_back({std::move(path), 0.0, completion, 0.0});
    }
  }
  return candidates;
}

} // namespace local_planning
