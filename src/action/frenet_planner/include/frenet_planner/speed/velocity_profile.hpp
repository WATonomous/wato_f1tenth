#ifndef FRENET_PLANNER_SPEED_VELOCITY_PROFILE_HPP
#define FRENET_PLANNER_SPEED_VELOCITY_PROFILE_HPP

#include "frenet_planner/core/types.hpp"
#include "frenet_planner/reference/raceline_reference.hpp"

#include <vector>

namespace frenet_planner
{

struct VelocityProfileConfig
{
  double friction_coeff = 1.0;
  double min_velocity_mps = 0.0;
  double max_velocity_mps = 10.0;
  double max_accel_mps2 = 5.0;
  double max_decel_mps2 = 5.0;
  double overtake_speed_scale = 1.1;  // OVERTAKE/PASS interior; MERGE terminal unscaled
};

struct VelocityProfileResult
{
  bool feasible = false;
  double traversal_time_s = 0.0;
};

// Feasible speed profile on a complete path; writes speeds only when feasible.
VelocityProfileResult assignVelocityProfile(
  std::vector<CurveSample> & path,
  double start_velocity_mps,
  double start_raceline_s,
  double terminal_raceline_s,
  PlannerIntent intent,
  const RacelineReference & reference,
  const VelocityProfileConfig & config);

} // namespace frenet_planner

#endif // FRENET_PLANNER_SPEED_VELOCITY_PROFILE_HPP
