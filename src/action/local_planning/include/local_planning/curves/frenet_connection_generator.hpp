#ifndef LOCAL_PLANNING_CURVES_FRENET_CONNECTION_GENERATOR_HPP
#define LOCAL_PLANNING_CURVES_FRENET_CONNECTION_GENERATOR_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/curves/frenet_polynomial.hpp"
#include "local_planning/reference/reference_window.hpp"

#include <cstddef>
#include <vector>

namespace local_planning
{

struct FrenetBoundary
{
  double s = 0.0;
  double d = 0.0;
  double d_prime = 0.0;          // dd/ds_ref
  double d_double_prime = 0.0;   // d2d/ds_ref2
};

struct FrenetConnectionConfig
{
  double sample_spacing_m = 0.1;      // matched to costmap resolution in the node
  double max_curvature_inv_m = 1.74;  // kinematic cap (steering + wheelbase)
  double friction_coeff = 1.0;
  double max_path_angle_deg = 60.0;   // max deviation from reference tangent

  // min(steering stop, friction circle); v <= 0 uses kinematic cap only
  double allowedCurvature(double v) const;
};

struct FrenetConnectionResult
{
  bool valid = false;
  RejectReason reject_reason = RejectReason::NONE;
  double max_abs_d = 0.0;  // worst |d| appended this call; unset if rejected
};

// Samples d(s) against a prebuilt reference window; rejects inline on first bad sample.
class FrenetConnectionGenerator
{
public:
  FrenetConnectionGenerator() = default;
  explicit FrenetConnectionGenerator(const FrenetConnectionConfig & config)
  : config_(config) {}

  const FrenetConnectionConfig & config() const {return config_;}
  void setSampleSpacingM(double sample_spacing_m)
  {
    config_.sample_spacing_m = sample_spacing_m;
  }

  // Restores path size on rejection. speed_mps is start speed (conservative curvature cap).
  FrenetConnectionResult generate(
    const ReferenceWindow & window,
    std::size_t i_start,
    std::size_t i_end,
    const FrenetPolynomial & polynomial,
    std::vector<CurveSample> & path,
    double speed_mps = 0.0) const;

private:
  FrenetConnectionConfig config_;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_CURVES_FRENET_CONNECTION_GENERATOR_HPP
