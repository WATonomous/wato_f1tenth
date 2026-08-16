#include "local_planning/maneuvers/maneuver_builder.hpp"

#include "local_planning/curves/frenet_polynomial.hpp"
#include "worker_pool.hpp"

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
// Past this the heading error is effectively perpendicular to the reference and
// tan() stops being a usable encoding of it.  The generator's max_path_angle_deg
// rejects long before here; this only keeps the arithmetic finite.
constexpr double kMaxStartHeadingErrorRad = 1.5;

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

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

void validateConfig(const ManeuverConfig & config, const VehicleGeometry & vehicle_geometry)
{
  if (!std::isfinite(config.horizon_m) || config.horizon_m <= 0.0) {
    throw std::invalid_argument("horizon_m must be finite and positive");
  }

  requireNonEmpty(
    config.overtake_s_offsets_from_opponent_rear_m,
    "overtake_s_offsets_from_opponent_rear_m");
  requireNonEmpty(config.passing_d_magnitudes_m, "passing_d_magnitudes_m");
  requireNonEmpty(config.overtake_heading_offsets_rad, "overtake_heading_offsets_rad");
  requireNonEmpty(config.pass_transition_distances_m, "pass_transition_distances_m");
  requireNonEmpty(config.merge_completion_distances_m, "merge_completion_distances_m");

  requireFinite(
    config.overtake_s_offsets_from_opponent_rear_m,
    "overtake_s_offsets_from_opponent_rear_m");
  requireFinite(config.passing_d_magnitudes_m, "passing_d_magnitudes_m");
  requireFinite(config.overtake_heading_offsets_rad, "overtake_heading_offsets_rad");
  requireFinite(config.pass_transition_distances_m, "pass_transition_distances_m");
  requireFinite(config.merge_completion_distances_m, "merge_completion_distances_m");

  if (std::any_of(
      config.passing_d_magnitudes_m.begin(), config.passing_d_magnitudes_m.end(),
      [](double magnitude) {return magnitude <= 0.0;}))
  {
    throw std::invalid_argument("passing_d_magnitudes_m must contain only positive values");
  }
  if (!std::isfinite(vehicle_geometry.collision_radius_m) ||
    vehicle_geometry.collision_radius_m <= 0.0)
  {
    throw std::invalid_argument("collision_circle_radius_m must be finite and positive");
  }
  const double side_deadband = vehicle_geometry.fullWidthM();
  if (std::any_of(
      config.passing_d_magnitudes_m.begin(), config.passing_d_magnitudes_m.end(),
      [side_deadband](double magnitude) {return magnitude <= side_deadband;}))
  {
    throw std::invalid_argument(
      "passing_d_magnitudes_m must be greater than vehicle width (2 * collision radius)");
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

} // namespace

ManeuverBuilder::ManeuverBuilder(
  const RacelineReference & reference,
  const FrenetConnectionGenerator & curve_generator,
  ManeuverConfig config,
  VehicleGeometry vehicle_geometry)
: reference_(reference), curve_generator_(curve_generator), config_(std::move(config)),
  vehicle_geometry_(vehicle_geometry)
{
  validateConfig(config_, vehicle_geometry_);
  removeDuplicates(config_.overtake_s_offsets_from_opponent_rear_m);
  removeDuplicates(config_.passing_d_magnitudes_m);
  removeDuplicates(config_.overtake_heading_offsets_rad);
  removeDuplicates(config_.pass_transition_distances_m);
  std::sort(
    config_.pass_transition_distances_m.begin(), config_.pass_transition_distances_m.end(),
    std::greater<double>());
  removeDuplicates(config_.merge_completion_distances_m);
}

bool ManeuverBuilder::prepareWindow(double ego_s) const
{
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
  const double heading_error = wrapAngle(ego.heading - reference.heading);
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
    curve_generator_.generate(window_, i_start, i_end, polynomial, path);
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
  // have started.  (MERGE's longest completion distance and PASS recovery's
  // non-preferred full-horizon offsets both land here.)
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

std::optional<double> ManeuverBuilder::preferredOffset(double ego_d) const
{
  std::optional<double> preferred;
  double nearest = std::numeric_limits<double>::infinity();
  for (double d : offsets(sideOf(ego_d))) {
    const double distance = std::abs(d - ego_d);
    if (distance < nearest - kTolerance ||
      (std::abs(distance - nearest) <= kTolerance &&
      (!preferred || std::abs(d) < std::abs(*preferred))))
    {
      preferred = d;
      nearest = distance;
    }
  }
  return preferred;
}

int ManeuverBuilder::sideOf(double d) const
{
  if (std::abs(d) <= vehicle_geometry_.fullWidthM()) {
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
  if (!reference_.valid() || !prepareWindow(ego_s)) {
    return candidates;
  }
  FrenetBoundary start;
  if (!startBoundary(ego, ego_s, ego_d, start)) {
    return candidates;
  }
  const double horizon_s = reference_.wrapS(ego_s + config_.horizon_m);
  const std::vector<double> lateral_offsets = offsets(0);
  // The first leg is a function of (intermediate_s, intermediate_d,
  // heading_offset).  It does not depend on horizon_d, so each distinct first
  // leg is sampled once and reused as the prefix for every same-side horizon
  // offset.  The two waves are job lists on the persistent pool; enumeration
  // stays in the original nested-loop order because selection breaks ties on
  // first-seen.
  //
  // The old (curvature mode) axis of this product is gone: with d'' = 0 the two
  // modes were the same boundary, and it was only ever a hedge against the G2
  // solver failing to converge on one of them.
  struct FirstLeg
  {
    Path path;
    FrenetBoundary join;
    // Carried with the leg so each completion that reuses this prefix starts
    // its own accumulation from the prefix's worst |d|.
    double max_abs_d = 0.0;
    bool valid = false;
  };
  const std::size_t heading_count = config_.overtake_heading_offsets_rad.size();
  // The tail family attaches to a leg that arrives tangent to the offset lane,
  // so it needs the zero-heading first leg.  When the configured grid contains
  // one, reuse it; otherwise build a separate entry leg for the tails alone.
  const auto zero_heading = std::find_if(
    config_.overtake_heading_offsets_rad.begin(),
    config_.overtake_heading_offsets_rad.end(),
    [](double heading) {return std::abs(heading) <= kTolerance;});
  const bool has_zero_heading =
    zero_heading != config_.overtake_heading_offsets_rad.end();
  const std::size_t zero_heading_index = has_zero_heading ?
    static_cast<std::size_t>(zero_heading - config_.overtake_heading_offsets_rad.begin()) :
    0U;

  const std::size_t station_count = config_.overtake_s_offsets_from_opponent_rear_m.size();
  const std::size_t offset_count = lateral_offsets.size();
  std::vector<FirstLeg> first_legs(station_count * offset_count * heading_count);
  std::vector<FirstLeg> exact_entries(station_count * offset_count);

  struct Station
  {
    double intermediate_s = 0.0;
    double progress = 0.0;
    bool ok = false;
  };
  std::vector<Station> stations(station_count);
  for (std::size_t s = 0; s < station_count; ++s) {
    stations[s].intermediate_s = reference_.wrapS(
      opponent_rear_s + config_.overtake_s_offsets_from_opponent_rear_m[s]);
    stations[s].progress = reference_.deltaS(ego_s, stations[s].intermediate_s);
    stations[s].ok = stations[s].progress > 0.0 && stations[s].progress < config_.horizon_m;
  }

  struct FirstJob
  {
    FirstLeg * leg = nullptr;
    FrenetBoundary end;
  };
  std::vector<FirstJob> first_jobs;
  first_jobs.reserve(first_legs.size() + exact_entries.size());
  for (std::size_t s = 0; s < station_count; ++s) {
    if (!stations[s].ok) {
      continue;
    }
    for (std::size_t d = 0; d < offset_count; ++d) {
      for (std::size_t h = 0; h < heading_count; ++h) {
        FrenetBoundary intermediate;
        if (!boundary(
            stations[s].intermediate_s, lateral_offsets[d],
            config_.overtake_heading_offsets_rad[h], intermediate))
        {
          continue;
        }
        FirstLeg & leg = first_legs[(s * offset_count + d) * heading_count + h];
        first_jobs.push_back({&leg, intermediate});
      }
      if (!has_zero_heading) {
        FrenetBoundary intermediate;
        if (boundary(stations[s].intermediate_s, lateral_offsets[d], 0.0, intermediate)) {
          first_jobs.push_back({&exact_entries[s * offset_count + d], intermediate});
        }
      }
    }
  }
  parallelFor(first_jobs.size(), [&](std::size_t i) {
      FirstJob & job = first_jobs[i];
      job.leg->join = job.end;
      job.leg->valid = connect(job.leg->path, start, job.end, job.leg->max_abs_d);
    });

  struct CompletionJob
  {
    const FirstLeg * leg = nullptr;
    FrenetBoundary horizon;
    double intermediate_s = 0.0;
    double passing_d = 0.0;
    double terminal_d = 0.0;
    bool offset_tail = false;
    double tail_distance = 0.0;
  };
  std::vector<CompletionJob> completions;
  completions.reserve(station_count * offset_count * offset_count * heading_count);
  for (std::size_t s = 0; s < station_count; ++s) {
    if (!stations[s].ok) {
      continue;
    }
    for (std::size_t d = 0; d < offset_count; ++d) {
      for (std::size_t horizon_i = 0; horizon_i < offset_count; ++horizon_i) {
        // Both legs stay on one side of the raceline.  Selection ranks on
        // |passing_d| and relies on this: no candidate reaching it straddles.
        if (lateral_offsets[d] * lateral_offsets[horizon_i] <= 0.0) {
          continue;
        }
        FrenetBoundary horizon;
        if (!boundary(horizon_s, lateral_offsets[horizon_i], 0.0, horizon)) {
          continue;
        }
        for (std::size_t h = 0; h < heading_count; ++h) {
          const FirstLeg & leg = first_legs[(s * offset_count + d) * heading_count + h];
          if (!leg.valid) {
            continue;
          }
          completions.push_back({
              &leg, horizon, stations[s].intermediate_s,
              lateral_offsets[d], lateral_offsets[horizon_i], false, 0.0});
        }
      }

      const FirstLeg * const entry = has_zero_heading ?
        &first_legs[(s * offset_count + d) * heading_count + zero_heading_index] :
        &exact_entries[s * offset_count + d];
      if (entry->valid) {
        completions.push_back({
            entry, FrenetBoundary{}, stations[s].intermediate_s,
            lateral_offsets[d], lateral_offsets[d], true,
            config_.horizon_m - stations[s].progress});
      }
    }
  }

  std::vector<ManeuverCandidate> slots(completions.size());
  std::vector<char> slot_ok(completions.size(), 0);
  parallelFor(completions.size(), [&](std::size_t i) {
      const CompletionJob & job = completions[i];
      Path path = job.leg->path;
      double max_abs_d = job.leg->max_abs_d;
      const bool ok = job.offset_tail ?
      appendOffsetTail(path, job.intermediate_s, job.tail_distance, job.passing_d, max_abs_d) :
      connect(path, job.leg->join, job.horizon, max_abs_d);
      if (!ok) {
        return;
      }
      slots[i] = {
        std::move(path), job.passing_d, job.terminal_d, config_.horizon_m,
        max_abs_d, job.offset_tail};
      slot_ok[i] = 1;
    });

  candidates.reserve(slots.size());
  for (std::size_t i = 0; i < slots.size(); ++i) {
    if (slot_ok[i]) {
      candidates.push_back(std::move(slots[i]));
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
  if (!reference_.valid() || side == 0 || !prepareWindow(ego_s)) {
    return candidates;
  }
  FrenetBoundary start;
  if (!startBoundary(ego, ego_s, ego_d, start)) {
    return candidates;
  }
  const double target_s = reference_.wrapS(ego_s + config_.horizon_m);
  const std::optional<double> target_d = preferredOffset(ego_d);
  if (!target_d) {
    return candidates;
  }
  FrenetBoundary target;
  Path path;
  double max_abs_d = 0.0;
  if (boundary(target_s, *target_d, 0.0, target) && connect(path, start, target, max_abs_d)) {
    if (staysOnSide(path, side)) {
      candidates.push_back(
        {std::move(path), *target_d, *target_d, config_.horizon_m, max_abs_d, false});
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
  if (!reference_.valid() || side == 0 || !prepareWindow(ego_s)) {
    return candidates;
  }
  FrenetBoundary start;
  if (!startBoundary(ego, ego_s, ego_d, start)) {
    return candidates;
  }
  const std::optional<double> preferred = preferredOffset(ego_d);
  if (!preferred) {
    return candidates;
  }
  const std::vector<double> allowed_offsets = offsets(side);
  for (double transition : config_.pass_transition_distances_m) {
    for (double d : allowed_offsets) {
      const bool targets_preferred = std::abs(d - *preferred) <= kTolerance;
      // The nominal PASS candidate already connects to preferred_d over the
      // full horizon.  Shorter preferred connections are new recovery options;
      // the full-horizon instance would only duplicate the nominal geometry.
      if (targets_preferred &&
        std::abs(transition - config_.horizon_m) <= kTolerance)
      {
        continue;
      }
      const double target_s = reference_.wrapS(ego_s + transition);
      FrenetBoundary target;
      Path path;
      double max_abs_d = 0.0;
      if (boundary(target_s, d, 0.0, target) &&
        connect(path, start, target, max_abs_d) &&
        appendOffsetTail(path, target_s, config_.horizon_m - transition, d, max_abs_d))
      {
        if (staysOnSide(path, side)) {
          candidates.push_back(
            {std::move(path), d, d, transition, max_abs_d,
              transition < config_.horizon_m - kTolerance});
        }
      }
    }
  }
  return candidates;
}

std::vector<ManeuverCandidate> ManeuverBuilder::merge(
  const BoundaryState & ego,
  double ego_s,
  double ego_d) const
{
  std::vector<ManeuverCandidate> candidates;
  if (!reference_.valid() || !prepareWindow(ego_s)) {
    return candidates;
  }
  FrenetBoundary start;
  if (!startBoundary(ego, ego_s, ego_d, start)) {
    return candidates;
  }
  for (double completion : config_.merge_completion_distances_m) {
    const double target_s = reference_.wrapS(ego_s + completion);
    FrenetBoundary target;
    Path path;
    double max_abs_d = 0.0;
    if (boundary(target_s, 0.0, 0.0, target) &&
      connect(path, start, target, max_abs_d) &&
      appendOffsetTail(path, target_s, config_.horizon_m - completion, 0.0, max_abs_d))
    {
      candidates.push_back(
        {std::move(path), 0.0, 0.0, completion, max_abs_d, false});
    }
  }
  return candidates;
}

} // namespace local_planning
