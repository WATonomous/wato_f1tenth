#ifndef LOCAL_PLANNING_CURVES_FRENET_POLYNOMIAL_HPP
#define LOCAL_PLANNING_CURVES_FRENET_POLYNOMIAL_HPP

namespace local_planning
{

// Lateral offset d(t), t = (s - s_start) / delta_s in [0, 1]. Derivative accessors are d/dt.
struct FrenetPolynomial
{
  double coeffs[6]{};
  double delta_s = 0.0;

  double evaluate(double t) const;
  double evaluateDerivative(double t) const;
  double evaluateSecondDerivative(double t) const;
};

// Quintic with matched position, slope, and curvature at both ends; monotone so max|d| = |D|.
FrenetPolynomial computeQuintic(
  double d_start,
  double slope_start,
  double second_derivative_start,
  double d_end,
  double slope_end,
  double second_derivative_end,
  double delta_s);

// d'' from vehicle curvature and reference geometry (steering-aware start boundary).
double frenetSecondDerivativeForVehicleCurvature(
  double vehicle_curvature,
  double lateral_offset,
  double lateral_slope,
  double reference_curvature,
  double reference_curvature_derivative);

} // namespace local_planning

#endif // LOCAL_PLANNING_CURVES_FRENET_POLYNOMIAL_HPP
