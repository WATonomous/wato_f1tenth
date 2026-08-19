#include "local_planning/curves/frenet_polynomial.hpp"

#include "local_planning/core/geometry.hpp"

#include <cmath>

namespace local_planning
{
namespace
{
constexpr double kEpsilon = kGridEps;
}  // namespace

double FrenetPolynomial::evaluate(double t) const
{
  return coeffs[0] + t *
         (coeffs[1] + t * (coeffs[2] + t * (coeffs[3] + t * (coeffs[4] + t * coeffs[5]))));
}

double FrenetPolynomial::evaluateDerivative(double t) const
{
  return coeffs[1] + t *
         (2.0 * coeffs[2] + t * (3.0 * coeffs[3] + t * (4.0 * coeffs[4] + t * 5.0 * coeffs[5])));
}

double FrenetPolynomial::evaluateSecondDerivative(double t) const
{
  return 2.0 * coeffs[2] + t * (6.0 * coeffs[3] + t * (12.0 * coeffs[4] + t * 20.0 * coeffs[5]));
}

FrenetPolynomial computeQuintic(
  double d_start,
  double slope_start,
  double second_derivative_start,
  double d_end,
  double slope_end,
  double second_derivative_end,
  double delta_s)
{
  // a0..a2 come straight from the start conditions; a3..a5 are the unique
  // solution of the three terminal conditions given those.
  const double a0 = d_start;
  const double a1 = slope_start * delta_s;
  const double a2 = 0.5 * second_derivative_start * delta_s * delta_s;

  const double position_residual = d_end - a0 - a1 - a2;
  const double slope_residual = slope_end * delta_s - a1 - 2.0 * a2;
  const double curvature_residual =
    second_derivative_end * delta_s * delta_s - 2.0 * a2;

  const double a3 =
    10.0 * position_residual - 4.0 * slope_residual + 0.5 * curvature_residual;
  const double a4 =
    -15.0 * position_residual + 7.0 * slope_residual - curvature_residual;
  const double a5 =
    6.0 * position_residual - 3.0 * slope_residual + 0.5 * curvature_residual;
  return {{a0, a1, a2, a3, a4, a5}, delta_s};
}

double frenetSecondDerivativeForVehicleCurvature(
  double vehicle_curvature,
  double lateral_offset,
  double lateral_slope,
  double reference_curvature,
  double reference_curvature_derivative)
{
  // For x(s) = r(s) + d(s) n(s), the Cartesian path curvature is
  // [k_ref A^2 + A d'' + k_ref' d d' + 2 k_ref d'^2] / (A^2 + d'^2)^(3/2),
  // where A = 1 - k_ref d.  Solve that for d''.
  const double tangent_scale = 1.0 - reference_curvature * lateral_offset;
  if (std::abs(tangent_scale) <= kEpsilon) {
    // The Frenet chart is singular here.  Fall back to the small-angle
    // conversion instead of amplifying numerical error
    return vehicle_curvature - reference_curvature;
  }

  const double tangent_norm_squared =
    tangent_scale * tangent_scale + lateral_slope * lateral_slope;
  const double tangent_norm_cubed =
    tangent_norm_squared * std::sqrt(tangent_norm_squared);
  return (
    vehicle_curvature * tangent_norm_cubed -
    reference_curvature * tangent_scale * tangent_scale -
    reference_curvature_derivative * lateral_offset * lateral_slope -
    2.0 * reference_curvature * lateral_slope * lateral_slope) /
         tangent_scale;
}

} // namespace local_planning
