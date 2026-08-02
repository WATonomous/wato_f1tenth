#ifndef LOCAL_PLANNING_CURVES_CLOTHOIDS_BACKEND_HPP
#define LOCAL_PLANNING_CURVES_CLOTHOIDS_BACKEND_HPP

// Private to the curve-generation layer.  Deliberately not under include/, and
// included only from curve_connection_generator.cpp, so that no Clothoids type
// ever reaches the public planner API.  Swapping this backend for
// steering_functions or a polynomial fit touches this pair of files and
// nothing else.

#include "local_planning/core/types.hpp"

#include <vector>

namespace local_planning::clothoids_backend
{

// Geometry only.  Speed is not the curve's business: samples come back with
// speed 0 and the velocity profile fills it later.
struct SolveResult
{
  std::vector<CurveSample> samples;
  double arc_length_m = 0.0;
  double max_abs_curvature_inv_m = 0.0;
  bool converged = false;
};


SolveResult solveG2(
  const BoundaryState & start,
  const BoundaryState & terminal,
  double sample_spacing_m);

} // namespace local_planning::clothoids_backend

#endif // LOCAL_PLANNING_CURVES_CLOTHOIDS_BACKEND_HPP
