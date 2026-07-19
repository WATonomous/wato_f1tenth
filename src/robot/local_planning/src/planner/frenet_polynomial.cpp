#include "planning/planner/frenet_polynomial.hpp"

namespace local_planning
{

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

FrenetPolynomial computeQuartic(
  double d_start,
  double slope_start,
  double second_derivative_start,
  double d_end,
  double slope_end,
  double delta_s)
{
  const double a0 = d_start;
  const double a1 = slope_start * delta_s;
  const double a2 = 0.5 * second_derivative_start * delta_s * delta_s;
  const double position_residual = d_end - a0 - a1 - a2;
  const double slope_residual = slope_end * delta_s - a1 - 2.0 * a2;

  const double a3 = 4.0 * position_residual - slope_residual;
  const double a4 = slope_residual - 3.0 * position_residual;
  return {{a0, a1, a2, a3, a4, 0.0}, delta_s};
}

FrenetPolynomial computeCubic(
  double d_start,
  double slope_start,
  double d_end,
  double slope_end,
  double delta_s)
{
  const double a0 = d_start;
  const double a1 = slope_start * delta_s;
  const double position_residual = d_end - a0 - a1;
  const double slope_residual = slope_end * delta_s - a1;

  const double a2 = 3.0 * position_residual - slope_residual;
  const double a3 = slope_residual - 2.0 * position_residual;
  return {{a0, a1, a2, a3, 0.0, 0.0}, delta_s};
}

} // namespace local_planning
