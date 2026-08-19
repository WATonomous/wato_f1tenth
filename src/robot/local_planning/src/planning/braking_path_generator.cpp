#include "local_planning/planning/braking_path_generator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>


/*
all this does is brake and drive towards the raceline in the safest possible path and we brute force
some possible paths 

*/
namespace local_planning
{
namespace
{
constexpr double kGravity = 9.81;
constexpr double kMinCarrotDistanceM = 1e-3;
constexpr double kMinFrenetJacobianMagnitude = 0.1;
constexpr double kStationToleranceM = 1e-4;
constexpr int kMaxStationCorrections = 2;

double sinc(double value)
{
  if (std::abs(value) < 1e-6) {
    const double value_sq = value * value;
    return 1.0 - value_sq / 6.0;
  }
  return std::sin(value) / value;
}
}  // namespace

double BrakingConfig::allowedCurvature(double v) const
{
  const double steering_limit = wheelbase_m > 0.0 ?
    std::tan(std::abs(max_steering_angle_rad)) / wheelbase_m :
    0.0;
  if (v <= 0.0) {return steering_limit;}
  return std::min(steering_limit, friction_coeff * kGravity / (v * v));
}

BrakingPathGenerator::BrakingPathGenerator(
  const RacelineReference & reference,
  BrakingConfig config)
: reference_(reference), config_(config)
{
}

std::vector<ManeuverCandidate> BrakingPathGenerator::generate(
  const BoundaryState & ego, double ego_s, double ego_d) const
{
  std::vector<ManeuverCandidate> candidates;
  if (!reference_.valid() || config_.sample_spacing_m <= 0.0 || config_.horizon_m <= 0.0) {
    return candidates;
  }
  const auto params = arcParams();
  candidates.reserve(params.size());
  for (const auto & item : params) {
    candidates.push_back(integrate(ego, ego_s, ego_d, item));
  }
  return candidates;
}

std::vector<BrakingArcParams> BrakingPathGenerator::arcParams() const
{
  std::vector<BrakingArcParams> params;
  params.reserve(config_.effort_levels.size() * config_.pursuit_lookaheads_m.size());
  for (const double effort : config_.effort_levels) {
    for (const double lookahead_m : config_.pursuit_lookaheads_m) {
      params.push_back({std::clamp(effort, 0.0, 1.0), lookahead_m});
    }
  }
  return params;
}

ManeuverCandidate BrakingPathGenerator::integrate(
  const BoundaryState & ego, double ego_s, double ego_d,
  const BrakingArcParams & params) const
{
  const double ds = config_.sample_spacing_m;
  // Inclusive of the horizon: the published path must stay longer than the
  // controller's largest lookahead, or find_lookahead() returns nothing and the
  // car dead-stops by a different route than the one this family removes.
  const auto count = static_cast<std::size_t>(std::ceil(config_.horizon_m / ds)) + 1;

  ManeuverCandidate candidate;
  candidate.path.reserve(count);
  candidate.uses_offset_tail = false;
  candidate.maneuver_distance_m = config_.horizon_m;

  double x = ego.x;
  double y = ego.y;
  double heading = ego.heading;
  double station = reference_.wrapS(ego_s);
  double lateral_offset = ego_d;

  for (std::size_t i = 0; i < count; ++i) {
    const double sigma = static_cast<double>(i) * ds;
    const double v = std::max(
      config_.min_velocity_mps,
      std::sqrt(std::max(0.0, ego.speed * ego.speed - 2.0 * config_.decel_mps2 * sigma)));

    if (i > 0) {
      station = reference_.wrapS(station + ds);
      double tangential = 0.0;
      ReferenceGeometrySample ref;
      for (int correction = 0; correction < kMaxStationCorrections; ++correction) {
        ref = reference_.sampleAtS(station);
        const double px = x - ref.x;
        const double py = y - ref.y;
        tangential = px * ref.tangent_x + py * ref.tangent_y;
        lateral_offset = px * ref.normal_x + py * ref.normal_y;
        if (std::abs(tangential) <= kStationToleranceM) {break;}

        double jacobian = 1.0 - ref.curvature * lateral_offset;
        if (std::abs(jacobian) < kMinFrenetJacobianMagnitude) {
          jacobian = std::copysign(kMinFrenetJacobianMagnitude, jacobian);
        }
        const double denominator = ref.parameter_speed * jacobian;
        if (!std::isfinite(denominator) || std::abs(denominator) < 1e-12) {break;}
        const double correction_m = std::clamp(tangential / denominator, -ds, ds);
        station = reference_.wrapS(station + correction_m);
      }

      ref = reference_.sampleAtS(station);
      const double px = x - ref.x;
      const double py = y - ref.y;
      tangential = px * ref.tangent_x + py * ref.tangent_y;
      lateral_offset = px * ref.normal_x + py * ref.normal_y;
      if (std::abs(tangential) > kStationToleranceM) {
        const Projection projected = reference_.project(Point(x, y), heading, station);
        station = projected.s;
        lateral_offset = projected.d;
      }
    }

    // Sliding carrot: the aim point is recomputed from this sample's own
    // station every step, so it keeps retreating ahead and the arc curls onto
    // the line and straightens rather than spearing a fixed point and
    // overshooting past it.
    const Point target = reference_.toCartesian(
      reference_.wrapS(station + params.lookahead_m), 0.0);
    const double dx = target.x - x;
    const double dy = target.y - y;
    const double distance_sq = dx * dx + dy * dy;
    double desired = 0.0;
    if (distance_sq > kMinCarrotDistanceM * kMinCarrotDistanceM) {
      // The same law pure pursuit steers by, on the true Euclidean distance
      // rather than the nominal lookahead, so a large lateral error is felt.
      const double lateral = -std::sin(heading) * dx + std::cos(heading) * dy;
      desired = 2.0 * lateral / distance_sq;
    }
    const double budget = params.effort * config_.allowedCurvature(v);
    const double curvature = std::clamp(desired, -budget, budget);

    CurveSample sample;
    sample.s = sigma;
    sample.x = x;
    sample.y = y;
    sample.heading = heading;
    sample.curvature = curvature;
    sample.speed = v;
    sample.raceline_s = station;
    sample.d = lateral_offset;
    candidate.path.push_back(sample);
    candidate.max_abs_d_m = std::max(candidate.max_abs_d_m, std::abs(lateral_offset));
    candidate.terminal_d = lateral_offset;
    candidate.passing_d = lateral_offset;

    const double half_turn = 0.5 * curvature * ds;
    const double chord = ds * sinc(half_turn);
    x += chord * std::cos(heading + half_turn);
    y += chord * std::sin(heading + half_turn);
    heading += curvature * ds;
  }
  return candidate;
}

}  // namespace local_planning
