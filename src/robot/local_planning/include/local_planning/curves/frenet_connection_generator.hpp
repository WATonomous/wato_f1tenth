#ifndef LOCAL_PLANNING_CURVES_FRENET_CONNECTION_GENERATOR_HPP
#define LOCAL_PLANNING_CURVES_FRENET_CONNECTION_GENERATOR_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/curves/frenet_polynomial.hpp"
#include "local_planning/reference/reference_window.hpp"

#include <cstddef>
#include <vector>

namespace local_planning
{

// The complete state a Frenet connection is anchored to at one end: a station
// on the reference and the first three terms of the lateral offset there.
// Derivatives are with respect to *reference* arc length.
struct FrenetBoundary
{
  double s = 0.0;
  double d = 0.0;
  double d_prime = 0.0;          // dd/ds_ref
  double d_double_prime = 0.0;   // d2d/ds_ref2
};

struct FrenetConnectionConfig
{
  // Forced to the live costmap resolution by the node: planning cannot usefully
  // sample finer than the grid the paths are checked against.
  double sample_spacing_m = 0.1;
  // Kinematic cap: 0.52 rad of steering over a 0.33 m wheelbase.
  double max_curvature_inv_m = 1.74;
  // How far the path may turn away from the reference tangent.  This is what
  // stops a connection doubling back, which is the job the clothoid family's
  // arc-length cap was standing in for -- badly, since it bounded total length
  // rather than direction.
  double max_path_angle_deg = 60.0;
};

struct FrenetConnectionResult
{
  bool valid = false;
  RejectReason reject_reason = RejectReason::NONE;
  // Worst |d| over the samples this call appended, accumulated in the sampling
  // loop rather than by a second sweep of the finished path.  Only set when
  // valid: a rejected connection restores the path and reports nothing.
  double max_abs_d = 0.0;
};

// Samples one d(s) polynomial against a prebuilt reference window, appending
// Cartesian samples to a path.
//
// Rejects inline: an infeasible connection bails at the first bad sample
// instead of paying for the full sweep and being thrown away afterwards, which
// is what the clothoid path did (it computed max curvature over the whole curve
// before deciding).
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

  // Appends grid indices [i_start, i_end] of the window to path, skipping
  // i_start when path is already non-empty so legs concatenate without a
  // duplicated join sample.  On rejection path is restored to its prior size,
  // so a partially sampled connection never leaks into a candidate.
  FrenetConnectionResult generate(
    const ReferenceWindow & window,
    std::size_t i_start,
    std::size_t i_end,
    const FrenetPolynomial & polynomial,
    std::vector<CurveSample> & path) const;

private:
  FrenetConnectionConfig config_;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_CURVES_FRENET_CONNECTION_GENERATOR_HPP
