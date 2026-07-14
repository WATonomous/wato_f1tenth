#include "planning/planner/quintic_polynomial.hpp"

namespace local_planning
{

double QuinticPolynomial::evaluate(double t) const
{
  return coeffs[0] + t *
         (coeffs[1] + t * (coeffs[2] + t * (coeffs[3] + t * (coeffs[4] + t * coeffs[5]))));
}

double QuinticPolynomial::evaluateDerivative(double t) const
{
  return coeffs[1] + t *
         (2.0 * coeffs[2] + t * (3.0 * coeffs[3] + t * (4.0 * coeffs[4] + t * 5.0 * coeffs[5])));
}

double QuinticPolynomial::evaluateSecondDerivative(double t) const
{
  return 2.0 * coeffs[2] + t * (6.0 * coeffs[3] + t * (12.0 * coeffs[4] + t * 20.0 * coeffs[5]));
}

QuinticPolynomial computeQuintic(
  double d_start,
  double slope_start,
  double second_derivative_start,
  double d_end,
  double slope_end,
  double second_derivative_end,
  double delta_s)
{
  const double a0 = d_start;
  const double a1 = slope_start * delta_s;
  const double a2 = 0.5 * second_derivative_start * delta_s * delta_s;
  const double position_residual = d_end - a0 - a1 - a2;
  const double slope_residual = slope_end * delta_s - a1 - 2.0 * a2;
  const double curvature_residual =
    second_derivative_end * delta_s * delta_s - 2.0 * a2;

  const double a3 = 10.0 * position_residual - 4.0 * slope_residual +
    0.5 * curvature_residual;
  const double a4 = -15.0 * position_residual + 7.0 * slope_residual -
    curvature_residual;
  const double a5 = 6.0 * position_residual - 3.0 * slope_residual +
    0.5 * curvature_residual;
  return {{a0, a1, a2, a3, a4, a5}, delta_s};
}

} // namespace local_planning
