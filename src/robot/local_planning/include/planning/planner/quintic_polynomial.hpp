#ifndef PLANNING_PLANNER_QUINTIC_POLYNOMIAL_HPP
#define PLANNING_PLANNER_QUINTIC_POLYNOMIAL_HPP

namespace local_planning
{

struct QuinticPolynomial
{
  double coeffs[6]{};
  double delta_s = 0.0;   //forward distance along the raceline for the corresponding edge

  double evaluate(double t) const;
  double evaluateDerivative(double t) const;
  double evaluateSecondDerivative(double t) const;
};

QuinticPolynomial computeQuintic(
  double d_start,
  double slope_start,
  double d_end,
  double slope_end,
  double delta_s);

} // namespace local_planning

#endif // PLANNING_PLANNER_QUINTIC_POLYNOMIAL_HPP
