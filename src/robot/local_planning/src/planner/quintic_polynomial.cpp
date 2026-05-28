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
  double d_end,
  double slope_end,
  double delta_s)
{
  const double a0 = d_start;
  const double a1 = slope_start * delta_s;
  const double a2 = 0.0;
  const double d = d_end - d_start - a1;
  const double h = (slope_end - slope_start) * delta_s;

  const double a3 = 10.0 * d - 4.0 * h;
  const double a4 = 7.0 * h - 15.0 * d;
  const double a5 = 6.0 * d - 3.0 * h;
  return {{a0, a1, a2, a3, a4, a5}, delta_s};
}

} // namespace local_planning
