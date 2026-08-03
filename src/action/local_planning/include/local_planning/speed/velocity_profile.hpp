#ifndef LOCAL_PLANNING_SPEED_VELOCITY_PROFILE_HPP
#define LOCAL_PLANNING_SPEED_VELOCITY_PROFILE_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/reference/raceline_reference.hpp"

#include <vector>

namespace local_planning
{

struct VelocityProfileResult
{
  bool feasible = false;
  double traversal_time_s = 0.0;
};

// Builds a dynamically feasible speed profile on a complete candidate path.
// Projects samples onto the raceline, applies intent-scaled raceline / vehicle /
// friction caps, an explicit terminal speed policy, then backward deceleration
// and forward acceleration passes.  Speeds are written to `path` only when the
// profile is feasible.
VelocityProfileResult assignVelocityProfile(
  std::vector<CurveSample> & path,
  double start_velocity_mps,
  double start_raceline_s,
  double terminal_raceline_s,
  PlannerIntent intent,
  const RacelineReference & reference,
  const LocalPlannerConfig & config);

} // namespace local_planning

#endif // LOCAL_PLANNING_SPEED_VELOCITY_PROFILE_HPP
