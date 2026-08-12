#ifndef LOCAL_PLANNING_CURVES_REFERENCE_CURVE_SAMPLER_HPP
#define LOCAL_PLANNING_CURVES_REFERENCE_CURVE_SAMPLER_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/reference/raceline_reference.hpp"

#include <vector>

namespace local_planning
{

struct ReferenceCurveRequest
{
  double start_s = 0.0;
  // Distance along the raceline parameter, not Cartesian offset-curve length.
  double reference_distance_m = 0.0;
  double d = 0.0;
  double sample_spacing_m = 0.1;
};

struct GeneratedReferenceCurve
{
  // Includes the exact start boundary. Sample s is Cartesian arc length from
  // that boundary; raceline_s retains the corresponding wrapped station.
  std::vector<CurveSample> samples;
  bool valid = false;
};

// Densely samples the raceline itself (d = 0) or one of its exact
// constant-Frenet-offset curves. This generator owns no maneuver policy and
// performs no concatenation with incoming connections.
class ReferenceCurveSampler
{
public:
  GeneratedReferenceCurve generate(
    const RacelineReference & reference,
    const ReferenceCurveRequest & request) const;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_CURVES_REFERENCE_CURVE_SAMPLER_HPP
