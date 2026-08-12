#include "local_planning/curves/reference_curve_sampler.hpp"

#include <algorithm>
#include <cmath>

namespace local_planning
{
namespace
{

constexpr double kTolerance = 1e-6;

bool sampleAt(
  const RacelineReference & reference,
  double raceline_s,
  double d,
  CurveSample & result)
{
  const ReferenceGeometrySample sample = reference.sampleAtS(raceline_s);
  const double denominator = 1.0 - d * sample.curvature;
  if (!std::isfinite(denominator) || denominator <= kTolerance) {
    return false;
  }

  result.x = sample.x + d * sample.normal_x;
  result.y = sample.y + d * sample.normal_y;
  result.heading = sample.heading;
  result.curvature = sample.curvature / denominator;
  result.speed = 0.0;
  result.raceline_s = sample.s_wrapped;
  return std::isfinite(result.x) && std::isfinite(result.y) &&
         std::isfinite(result.heading) && std::isfinite(result.curvature) &&
         std::isfinite(result.raceline_s);
}

}  // namespace

GeneratedReferenceCurve ReferenceCurveSampler::generate(
  const RacelineReference & reference,
  const ReferenceCurveRequest & request) const
{
  GeneratedReferenceCurve result;
  if (!reference.valid() || !std::isfinite(request.start_s) ||
    !std::isfinite(request.reference_distance_m) ||
    request.reference_distance_m < 0.0 || !std::isfinite(request.d) ||
    !std::isfinite(request.sample_spacing_m) || request.sample_spacing_m <= 0.0)
  {
    return result;
  }

  CurveSample start;
  if (!sampleAt(reference, request.start_s, request.d, start)) {
    return result;
  }
  start.s = 0.0;
  result.samples.push_back(start);

  double covered = 0.0;
  while (covered < request.reference_distance_m) {
    covered += std::min(
      request.sample_spacing_m, request.reference_distance_m - covered);
    CurveSample next;
    if (!sampleAt(reference, request.start_s + covered, request.d, next)) {
      result.samples.clear();
      return result;
    }
    const CurveSample & previous = result.samples.back();
    next.s = previous.s + std::hypot(next.x - previous.x, next.y - previous.y);
    result.samples.push_back(next);
  }

  result.valid = true;
  return result;
}

}  // namespace local_planning
