#ifndef LOCAL_PLANNING_SPEED_VELOCITY_PROFILE_HPP
#define LOCAL_PLANNING_SPEED_VELOCITY_PROFILE_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/reference/raceline_reference.hpp"

#include <vector>

namespace local_planning
{

struct VelocityProfileConfig
{
  double friction_coeff = 1.0;
  double min_velocity_mps = 0.0;
  double max_velocity_mps = 10.0;
  double max_accel_mps2 = 5.0;
  double max_decel_mps2 = 5.0;
  // Floor speed on braking-fallback paths only. Maneuver profiles still use
  // min_velocity_mps. Keeps pure pursuit from dead-stopping when no candidate
  // wins selection but a collision-free slow-along path exists.
  double braking_fallback_min_velocity_mps = 1.0;
  // Interior/terminal racing-speed multiplier for OVERTAKE/PASS, and MERGE
  // interiors only. MERGE's horizon terminal stays at unscaled raceline speed.
  double overtake_speed_scale = 1.1;
};

struct VelocityProfileResult
{
  bool feasible = false;
  double traversal_time_s = 0.0;
};

// Builds a dynamically feasible speed profile on a complete candidate path.
// Uses the raceline station retained during maneuver construction, applies
// intent-scaled raceline / vehicle / friction caps, an explicit terminal speed policy, then backward deceleration
// and forward acceleration passes.  Speeds are written to `path` only when the
// profile is feasible.
VelocityProfileResult assignVelocityProfile(
  std::vector<CurveSample> & path,
  double start_velocity_mps,
  double start_raceline_s,
  double terminal_raceline_s,
  PlannerIntent intent,
  const RacelineReference & reference,
  const VelocityProfileConfig & config);

} // namespace local_planning

#endif // LOCAL_PLANNING_SPEED_VELOCITY_PROFILE_HPP
