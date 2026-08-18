#include "local_planning/curves/frenet_connection_generator.hpp"

#include <algorithm>
#include <cmath>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;
constexpr double kGravityMps2 = 9.81;

FrenetConnectionResult rejected(RejectReason reason)
{
  return {false, reason};
}

}  // namespace

double FrenetConnectionConfig::allowedCurvature(double v) const
{
  if (!(v > 0.0) || !std::isfinite(v)) {
    return max_curvature_inv_m;
  }
  return std::min(max_curvature_inv_m, friction_coeff * kGravityMps2 / (v * v));
}

FrenetConnectionResult FrenetConnectionGenerator::generate(
  const ReferenceWindow & window,
  std::size_t i_start,
  std::size_t i_end,
  const FrenetPolynomial & polynomial,
  std::vector<CurveSample> & path,
  double speed_mps) const
{
  if (!window.valid() || i_end <= i_start || i_end >= window.size() ||
    !(polynomial.delta_s > kEpsilon))
  {
    return rejected(RejectReason::CHART_SINGULAR);
  }

  const std::size_t restore_to = path.size();
  const bool skip_first = !path.empty();
  const double s_offset = path.empty() ? 0.0 : path.back().s;
  const double delta_s = polynomial.delta_s;
  const double inverse_delta_s = 1.0 / delta_s;
  const double inverse_delta_s_sq = inverse_delta_s * inverse_delta_s;
  const double step_ratio = 1.0 / static_cast<double>(i_end - i_start);
  const double reference_step = window.spacingM();
  const double max_path_angle_rad = config_.max_path_angle_deg * kPi / 180.0;
  const double max_curvature = config_.allowedCurvature(speed_mps);

  const auto reject = [&](RejectReason reason) {
      path.resize(restore_to);
      return rejected(reason);
    };

  // Trapezoidal accumulation of the exact differential
  // ds_path = sqrt(A^2 + d'^2) ds_ref, reusing the n2 the curvature needs
  // anyway.  s must stay *true path* arc length: the velocity profile
  // differences it for the accel/decel integration.
  double path_s = s_offset;
  double previous_speed_factor = 0.0;
  double max_abs_d = 0.0;

  for (std::size_t i = i_start; i <= i_end; ++i) {
    const ReferenceGeometrySample & reference = window.at(i);
    const double t = static_cast<double>(i - i_start) * step_ratio;

    const double d = polynomial.evaluate(t);
    const double d_prime = polynomial.evaluateDerivative(t) * inverse_delta_s;
    const double d_double_prime =
      polynomial.evaluateSecondDerivative(t) * inverse_delta_s_sq;

    const double tangent_scale = 1.0 - reference.curvature * d;
    if (!(tangent_scale > kEpsilon) || !std::isfinite(tangent_scale)) {
      return reject(RejectReason::CHART_SINGULAR);
    }

    const double angle = std::atan2(d_prime, tangent_scale);
    if (std::abs(angle) > max_path_angle_rad) {
      return reject(RejectReason::HEADING_LIMIT);
    }

    const double normal_term = reference.curvature * tangent_scale + d_double_prime;
    const double tangential_term =
      -reference.curvature_derivative * d - 2.0 * reference.curvature * d_prime;
    const double norm_sq = tangent_scale * tangent_scale + d_prime * d_prime;
    const double speed_factor = std::sqrt(norm_sq);
    const double curvature =
      (tangent_scale * normal_term - d_prime * tangential_term) / (norm_sq * speed_factor);
    if (!std::isfinite(curvature) || std::abs(curvature) > max_curvature) {
      return reject(RejectReason::CURVATURE_LIMIT);
    }

    if (i > i_start) {
      path_s += 0.5 * (previous_speed_factor + speed_factor) * reference_step;
    }
    previous_speed_factor = speed_factor;

    if (i == i_start && skip_first) {
      continue;
    }

    CurveSample sample;
    sample.s = path_s;
    sample.x = reference.x + d * reference.normal_x;
    sample.y = reference.y + d * reference.normal_y;
    sample.heading = reference.heading + angle;
    sample.curvature = curvature;
    sample.speed = 0.0;
    sample.raceline_s = reference.s_wrapped;
    sample.d = d;
    if (!std::isfinite(sample.x) || !std::isfinite(sample.y) ||
      !std::isfinite(sample.heading) || !std::isfinite(sample.s))
    {
      return reject(RejectReason::CHART_SINGULAR);
    }
    max_abs_d = std::max(max_abs_d, std::abs(d));
    path.push_back(sample);
  }

  return {true, RejectReason::NONE, max_abs_d};
}

}  // namespace local_planning
