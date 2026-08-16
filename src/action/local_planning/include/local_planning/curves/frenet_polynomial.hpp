#ifndef LOCAL_PLANNING_CURVES_FRENET_POLYNOMIAL_HPP
#define LOCAL_PLANNING_CURVES_FRENET_POLYNOMIAL_HPP

namespace local_planning
{

// A lateral-offset polynomial d(t) with t = (s - s_start) / delta_s in [0, 1],
// where delta_s is *reference* arc length.  POD, no allocation, Horner
// evaluation: this is the whole connection primitive.
//
// The derivative accessors return d/dt.  Callers divide by delta_s (and its
// square) to get derivatives with respect to reference arc length, which is
// what the curvature formula wants.
struct FrenetPolynomial
{
  double coeffs[6]{};
  double delta_s = 0.0;

  double evaluate(double t) const;
  double evaluateDerivative(double t) const;
  double evaluateSecondDerivative(double t) const;
};

// The only connection this planner builds: position, slope, and curvature are
// all pinned at both ends.  With matched boundary conditions the result is
// d(t) = D(10t^3 - 15t^4 + 6t^5), whose derivative 30D t^2 (1-t)^2 is
// non-negative everywhere -- strictly monotone, max|d| = |D| exactly.  That is
// the property the whole port rests on: there is no bulge to collide with.
FrenetPolynomial computeQuintic(
  double d_start,
  double slope_start,
  double second_derivative_start,
  double d_end,
  double slope_end,
  double second_derivative_end,
  double delta_s);

// Inverts the Frenet->Cartesian curvature relation for d'', so a connection can
// start by continuing the curvature the current steering angle implies rather
// than assuming the car is already tracking the reference.
double frenetSecondDerivativeForVehicleCurvature(
  double vehicle_curvature,
  double lateral_offset,
  double lateral_slope,
  double reference_curvature,
  double reference_curvature_derivative);

} // namespace local_planning

#endif // LOCAL_PLANNING_CURVES_FRENET_POLYNOMIAL_HPP
