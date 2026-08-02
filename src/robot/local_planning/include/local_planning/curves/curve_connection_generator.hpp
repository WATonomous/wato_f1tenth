#ifndef LOCAL_PLANNING_CURVES_CURVE_CONNECTION_GENERATOR_HPP
#define LOCAL_PLANNING_CURVES_CURVE_CONNECTION_GENERATOR_HPP

#include "local_planning/core/types.hpp"

#include <vector>

namespace local_planning
{


struct ConnectionRequest
{
  BoundaryState start;

  // Use raceline curvature at the horizon and offset-lane
  // curvature kappa / (1 - d * kappa) at an intermediate.
  BoundaryState terminal;
};

struct GeneratedConnection
{
  std::vector<CurveSample> samples;   // dense, s from 0 at the start boundary
  BoundaryState actual_terminal;      // what the solver actually produced
  bool valid = false;
  RejectReason reject_reason = RejectReason::NONE;
};

// The G2 solver bounds neither curvature nor arc length, so a converged
// solution can still be undriveable or loop back on itself.
struct CurveGeneratorConfig
{
  double sample_spacing_m = 0.1;
  // Kinematic cap: 0.52 rad of steering over a 0.33 m wheelbase.
  double max_curvature_inv_m = 1.74;
 //better way might be to just check if it becomes antiparallel with raceline
  double max_arc_length_m = 12.0;
};


class CurveConnectionGenerator
{
public:
  CurveConnectionGenerator() = default;
  explicit CurveConnectionGenerator(const CurveGeneratorConfig & config)
  : config_(config) {}

  const CurveGeneratorConfig & config() const {return config_;}

  GeneratedConnection generate(const ConnectionRequest & request) const;

private:
  CurveGeneratorConfig config_;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_CURVES_CURVE_CONNECTION_GENERATOR_HPP
