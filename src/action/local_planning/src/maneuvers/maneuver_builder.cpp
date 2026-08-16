#include "local_planning/maneuvers/maneuver_builder.hpp"

#include "local_planning/curves/reference_curve_sampler.hpp"
#include "worker_pool.hpp"

#include <algorithm>
#include <array>
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

bool ManeuverBuilder::boundary(
  double s,
  double d,
  double heading_offset,
  BoundaryState & result,
  BoundaryCurvature curvature) const
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
    curvature == BoundaryCurvature::REFERENCE ?
    reference.curvature : reference.curvature / denominator,
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

bool ManeuverBuilder::appendReferenceCurve(
  Path & path,
  double start_s,
  double reference_distance_m,
  double d) const
{
  if (path.empty() || curve_generator_.config().sample_spacing_m <= 0.0) {
    return false;
  }
  const GeneratedReferenceCurve generated = ReferenceCurveSampler().generate(
    reference_,
    {start_s, reference_distance_m, d, curve_generator_.config().sample_spacing_m});
  if (!generated.valid || generated.samples.empty()) {
    return false;
  }
  const CurveSample & first = generated.samples.front();
  const BoundaryState start{
    first.x, first.y, first.heading, first.curvature, first.speed};
  if (!matches(path.back(), start)) {
    return false;
  }

  const double s_offset = path.back().s;
  for (std::size_t i = 1; i < generated.samples.size(); ++i) {
    CurveSample sample = generated.samples[i];
    sample.s += s_offset;
    path.push_back(sample);
  }
  return true;
}

ManeuverBuilder::SideCheck ManeuverBuilder::sideAndDeviation(
  const Path & path,
  int side,
  bool allow_start_center,
  double target_d) const
{
  const double deadband = vehicle_geometry_.fullWidthM();
  SideCheck result;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const CurveSample & sample = path[i];
    // Every sample already knows its station -- connect() and the reference
    // curve sampler set raceline_s when they build it -- so this needs no
    // search.  The old
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
  if (!reference_.valid()) {
    return candidates;
  }
  // Unused: the dense side check is deferred to opponent prediction (review P0-1).
  // The old zero-curvature intermediate boundary that caused tight-corner crossings
  // has been removed; OVERTAKE now uses only reference and exact offset curvature.
  (void)ego_d;
  const double horizon_s = reference_.wrapS(ego_s + config_.horizon_m);
  const std::vector<double> lateral_offsets = offsets(0);
  // The first leg is a function of (intermediate_s, intermediate_d,
  // heading_offset, curvature mode).  It does not depend on horizon_d, so
  // each distinct first leg is solved once and reused as the prefix for every
  // same-side horizon offset.  The two G2 waves are job lists on the
  // persistent pool; enumeration stays in the original nested-loop order
  // because selectOvertake() breaks ties on first-seen.
  struct FirstLeg
  {
    Path path;
    BoundaryState join;
    bool valid = false;
  };
  constexpr std::array<BoundaryCurvature, 2> curvature_modes{
    BoundaryCurvature::REFERENCE,
    BoundaryCurvature::OFFSET};
  const std::size_t heading_count = config_.overtake_heading_offsets_rad.size();
  const std::size_t curvature_count = curvature_modes.size();
  const auto zero_heading = std::find_if(
    config_.overtake_heading_offsets_rad.begin(),
    config_.overtake_heading_offsets_rad.end(),
    [](double heading) {return std::abs(heading) <= kTolerance;});
  const std::optional<std::size_t> zero_heading_index =
    zero_heading == config_.overtake_heading_offsets_rad.end() ?
    std::nullopt :
    std::optional<std::size_t>(static_cast<std::size_t>(
        zero_heading - config_.overtake_heading_offsets_rad.begin()));

  const std::size_t station_count = config_.overtake_s_offsets_from_opponent_rear_m.size();
  const std::size_t offset_count = lateral_offsets.size();
  const std::size_t first_stride = heading_count * curvature_count;
  std::vector<FirstLeg> first_legs(station_count * offset_count * first_stride);
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
    BoundaryState end;
    double end_s = 0.0;
  };
  std::vector<FirstJob> first_jobs;
  first_jobs.reserve(first_legs.size() + exact_entries.size());
  for (std::size_t s = 0; s < station_count; ++s) {
    if (!stations[s].ok) {
      continue;
    }
    for (std::size_t d = 0; d < offset_count; ++d) {
      for (std::size_t h = 0; h < heading_count; ++h) {
        for (std::size_t c = 0; c < curvature_count; ++c) {
          BoundaryState intermediate;
          if (!boundary(
              stations[s].intermediate_s, lateral_offsets[d],
              config_.overtake_heading_offsets_rad[h], intermediate, curvature_modes[c]))
          {
            continue;
          }
          FirstLeg & leg = first_legs[(s * offset_count + d) * first_stride +
              h * curvature_count + c];
          first_jobs.push_back({&leg, intermediate, stations[s].intermediate_s});
        }
      }
      if (!zero_heading_index) {
        BoundaryState intermediate;
        if (boundary(
            stations[s].intermediate_s, lateral_offsets[d], 0.0, intermediate,
            BoundaryCurvature::OFFSET))
        {
          first_jobs.push_back({
              &exact_entries[s * offset_count + d], intermediate,
              stations[s].intermediate_s});
        }
      }
    }
  }
  parallelFor(first_jobs.size(), [&](std::size_t i) {
      FirstJob & job = first_jobs[i];
      job.leg->valid = connect(
        job.leg->path, ego, job.end, ego_s, job.end_s, &job.leg->join);
    });

  struct CompletionJob
  {
    const FirstLeg * leg = nullptr;
    BoundaryState horizon;
    double intermediate_s = 0.0;
    double horizon_s = 0.0;
    double target_d = 0.0;
    bool offset_tail = false;
    double tail_distance = 0.0;
  };
  std::vector<CompletionJob> completions;
  completions.reserve(station_count * offset_count * offset_count * first_stride);
  for (std::size_t s = 0; s < station_count; ++s) {
    if (!stations[s].ok) {
      continue;
    }
    for (std::size_t d = 0; d < offset_count; ++d) {
      for (std::size_t horizon_i = 0; horizon_i < offset_count; ++horizon_i) {
        if (lateral_offsets[d] * lateral_offsets[horizon_i] <= 0.0) {
          continue;
        }
        BoundaryState horizon;
        if (!boundary(horizon_s, lateral_offsets[horizon_i], 0.0, horizon)) {
          continue;
        }
        for (std::size_t h = 0; h < heading_count; ++h) {
          for (std::size_t c = 0; c < curvature_count; ++c) {
            const FirstLeg & leg = first_legs[(s * offset_count + d) * first_stride +
                h * curvature_count + c];
            if (!leg.valid) {
              continue;
            }
            completions.push_back({
                &leg, horizon, stations[s].intermediate_s, horizon_s,
                lateral_offsets[horizon_i], false, 0.0});
          }
        }
      }

      const FirstLeg * entry = nullptr;
      if (zero_heading_index) {
        entry = &first_legs[(s * offset_count + d) * first_stride +
            *zero_heading_index * curvature_count + 1U];
      } else {
        entry = &exact_entries[s * offset_count + d];
      }
      if (entry->valid) {
        completions.push_back({
            entry, BoundaryState{}, stations[s].intermediate_s, horizon_s,
            lateral_offsets[d], true, config_.horizon_m - stations[s].progress});
      }
    }
  }

  std::vector<ManeuverCandidate> slots(completions.size());
  std::vector<char> slot_ok(completions.size(), 0);
  parallelFor(completions.size(), [&](std::size_t i) {
      const CompletionJob & job = completions[i];
      Path path = job.leg->path;
      const bool ok = job.offset_tail ?
      appendReferenceCurve(path, job.intermediate_s, job.tail_distance, job.target_d) :
      connect(path, job.leg->join, job.horizon, job.intermediate_s, job.horizon_s);
      if (!ok) {
        return;
      }
      slots[i] = {std::move(path), job.target_d, config_.horizon_m, 0.0, job.offset_tail};
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
  if (!reference_.valid() || side == 0) {
    return candidates;
  }
  const double target_s = reference_.wrapS(ego_s + config_.horizon_m);
  const std::optional<double> target_d = preferredOffset(ego_d);
  if (!target_d) {
    return candidates;
  }
  BoundaryState target;
  Path path;
  if (boundary(target_s, *target_d, 0.0, target) &&
    connect(path, ego, target, ego_s, target_s))
  {
    const SideCheck check = sideAndDeviation(path, side, false, *target_d);
    if (check.stays_on_side) {
      candidates.push_back(
        {std::move(path), *target_d, config_.horizon_m, check.max_offset_deviation_m});
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
      BoundaryState target;
      Path path;
      if (boundary(target_s, d, 0.0, target) &&
        connect(path, ego, target, ego_s, target_s) &&
        appendReferenceCurve(path, target_s, config_.horizon_m - transition, d))
      {
        // Deviation is measured against the preferred offset, not this
        // candidate's own d: recovery candidates are ranked by how far they
        // stray from where the planner would rather be.
        const SideCheck check = sideAndDeviation(path, side, false, *preferred);
        if (check.stays_on_side) {
          candidates.push_back(
            {std::move(path), d, transition, check.max_offset_deviation_m,
              transition < config_.horizon_m - kTolerance});
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
      appendReferenceCurve(path, target_s, config_.horizon_m - completion, 0.0))
    {
      candidates.push_back({std::move(path), 0.0, completion, 0.0});
    }
  }
  return candidates;
}

} // namespace local_planning
