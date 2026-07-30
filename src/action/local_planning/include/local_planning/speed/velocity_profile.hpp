#ifndef LOCAL_PLANNING_SPEED_VELOCITY_PROFILE_HPP
#define LOCAL_PLANNING_SPEED_VELOCITY_PROFILE_HPP

#include "local_planning/core/types.hpp"

#include <vector>

namespace local_planning
{

// Clamps velocities, pins the start speed to measured odometry, then runs a
// backward deceleration pass and a forward acceleration pass.

// Carried forward from the DP planner's velocity_smoothing.  Phase 4 grows this
// file into the full profiler: the raceline baseline via RacelineReference, the
// intent scale, the curvature/friction cap, the terminal speed cap in all modes
// (PRD 12), and the predicted traversal time the selector compares on.
void smoothVelocityProfile(
  std::vector<Point> & path,
  double start_velocity_mps,
  const LocalPlannerConfig & config);

} // namespace local_planning

#endif // LOCAL_PLANNING_SPEED_VELOCITY_PROFILE_HPP
