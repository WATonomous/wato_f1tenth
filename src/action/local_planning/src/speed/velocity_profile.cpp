#include "local_planning/speed/velocity_profile.hpp"

#include "local_planning/core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = kGridEps;
constexpr double kGravityMps2 = 9.81;

double interiorSpeedScale(PlannerIntent intent, const VelocityProfileConfig & config)
{
  switch (intent) {
    case PlannerIntent::FOLLOW_RACING_LINE:
      return 1.0;
    case PlannerIntent::OVERTAKE:
    case PlannerIntent::PASS:
    case PlannerIntent::MERGE:
      return config.overtake_speed_scale;
  }
  return 1.0;
}

double terminalSpeedScale(PlannerIntent intent, const VelocityProfileConfig & config)
{
  // MERGE hands off to the global follower at unscaled raceline speed.
  if (intent == PlannerIntent::MERGE) {
    return 1.0;
  }
  return interiorSpeedScale(intent, config);
}

bool configValid(const VelocityProfileConfig & config)
{
  return std::isfinite(config.friction_coeff) &&
         config.friction_coeff > 0.0 &&
         std::isfinite(config.min_velocity_mps) &&
         config.min_velocity_mps >= 0.0 &&
         std::isfinite(config.max_velocity_mps) &&
         config.max_velocity_mps >= config.min_velocity_mps &&
         std::isfinite(config.max_accel_mps2) &&
         config.max_accel_mps2 > 0.0 &&
         std::isfinite(config.max_decel_mps2) &&
         config.max_decel_mps2 > 0.0 &&
         std::isfinite(config.overtake_speed_scale) &&
         config.overtake_speed_scale > 0.0;
}

bool finiteSample(const CurveSample & sample)
{
  return std::isfinite(sample.s) &&
         std::isfinite(sample.raceline_s) &&
         std::isfinite(sample.x) &&
         std::isfinite(sample.y) &&
         std::isfinite(sample.heading) &&
         std::isfinite(sample.curvature);
}

double frictionSpeedLimit(double curvature, const VelocityProfileConfig & config)
{
  const double abs_curvature = std::abs(curvature);
  if (abs_curvature <= kEpsilon) {
    return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(config.friction_coeff * kGravityMps2 / abs_curvature);
}

VelocityProfileResult reject()
{
  return {false, 0.0};
}

} // namespace

VelocityProfileResult assignVelocityProfile(
  std::vector<CurveSample> & path,
  double start_velocity_mps,
  double start_raceline_s,
  double terminal_raceline_s,
  PlannerIntent intent,
  const RacelineReference & reference,
  const VelocityProfileConfig & config)
{
  if (!configValid(config) || !reference.valid() || path.size() < 2) {
    return reject();
  }
  if (!std::isfinite(start_velocity_mps) ||
    !std::isfinite(start_raceline_s) ||
    !std::isfinite(terminal_raceline_s))
  {
    return reject();
  }

  for (const CurveSample & sample : path) {
    if (!finiteSample(sample)) {
      return reject();
    }
  }

  std::vector<double> segment_lengths(path.size() - 1, 0.0);
  bool has_progress = false;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double ds = path[i].s - path[i - 1].s;
    if (ds < -kEpsilon) {
      return reject();
    }
    segment_lengths[i - 1] = std::max(0.0, ds);
    if (ds > kEpsilon) {
      has_progress = true;
    }
  }
  if (!has_progress) {
    return reject();
  }

  const double scale = interiorSpeedScale(intent, config);
  const double terminal_scale = terminalSpeedScale(intent, config);
  std::vector<double> speeds(path.size(), 0.0);
  for (std::size_t i = 0; i < path.size(); ++i) {
    const CurveSample & sample = path[i];
    const double raceline_speed =
      std::max(0.0, reference.velocityAtS(sample.raceline_s));
    double speed = scale * raceline_speed;
    speed = std::max(config.min_velocity_mps, speed);
    speed = std::min(speed, config.max_velocity_mps);
    speed = std::min(speed, frictionSpeedLimit(sample.curvature, config));
    if (!std::isfinite(speed) || speed < 0.0) {
      return reject();
    }
    speeds[i] = speed;
  }

  const double terminal_raceline_speed =
    std::max(0.0, reference.velocityAtS(terminal_raceline_s));
  speeds.back() = std::min(speeds.back(), terminal_scale * terminal_raceline_speed);
  speeds.back() = std::min(speeds.back(), config.max_velocity_mps);
  speeds.back() = std::min(
    speeds.back(),
    frictionSpeedLimit(path.back().curvature, config));

  const double max_decel = config.max_decel_mps2;
  for (std::size_t i = path.size() - 1; i > 0; --i) {
    const std::size_t prev_index = i - 1;
    const double ds = segment_lengths[prev_index];
    if (ds <= kEpsilon) {
      speeds[prev_index] = std::min(speeds[prev_index], speeds[i]);
      continue;
    }

    const double next_velocity = speeds[i];
    const double allowed_velocity =
      std::sqrt(next_velocity * next_velocity + 2.0 * max_decel * ds);
    speeds[prev_index] = std::min(speeds[prev_index], allowed_velocity);
  }

/*
IMPORTANT: Make it like backward pass initial velocity must be 1m/s or smth
like that above my current velocity like it needs to be within a range to be valid
i dont like the current behaviour I think its dangerous
smth like this ]

if (start_velocity_mps > speeds[0] + tolerance) {
  return reject();
}
*/
  const double max_accel = config.max_accel_mps2;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double ds = segment_lengths[i - 1];
    if (ds <= kEpsilon) {
      speeds[i] = std::min(speeds[i], speeds[i - 1]);
      continue;
    }

    const double prev_velocity = speeds[i - 1];
    const double allowed_velocity =
      std::sqrt(prev_velocity * prev_velocity + 2.0 * max_accel * ds);
    speeds[i] = std::min(speeds[i], allowed_velocity);
  }

  double traversal_time_s = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double ds = segment_lengths[i - 1];
    if (ds <= kEpsilon) {
      continue;
    }

    const double average_speed = 0.5 * (speeds[i - 1] + speeds[i]);
    if (!std::isfinite(average_speed) || average_speed <= kEpsilon) {
      return reject();
    }
    traversal_time_s += ds / average_speed;
  }

  if (!std::isfinite(traversal_time_s) || traversal_time_s < 0.0) {
    return reject();
  }

  for (std::size_t i = 0; i < path.size(); ++i) {
    path[i].speed = speeds[i];
  }

  return {true, traversal_time_s};
}

} // namespace local_planning
