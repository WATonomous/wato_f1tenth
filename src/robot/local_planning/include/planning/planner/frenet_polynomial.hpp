#ifndef PLANNING_PLANNER_FRENET_POLYNOMIAL_HPP
#define PLANNING_PLANNER_FRENET_POLYNOMIAL_HPP

namespace local_planning
{

// A lateral-offset polynomial d(t) with t = s / delta_s in [0, 1].  These are
// the connection curves between lattice states: quartic and cubic connections
// differ only in which boundary derivatives they constrain, so one coefficient
// container covers both (unused high coefficients are zero).
struct FrenetPolynomial
{
  double coeffs[6]{};
  double delta_s = 0.0;   //forward distance along the raceline for the corresponding edge

  double evaluate(double t) const;
  double evaluateDerivative(double t) const;
  double evaluateSecondDerivative(double t) const;
};

// Measured-start connection: start d/d'/d'' come from the vehicle, the
// terminal d'' is left free so the transition can spread across the full
// delta_s.  Used for direct car-to-lattice edges and the smoother's first
// segment.
FrenetPolynomial computeQuartic(
  double d_start,
  double slope_start,
  double second_derivative_start,
  double d_end,
  double slope_end,
  double delta_s);

// Ordinary adjacent-layer lattice connection: d'' is unconstrained at both
// anchors.
FrenetPolynomial computeCubic(
  double d_start,
  double slope_start,
  double d_end,
  double slope_end,
  double delta_s);

} // namespace local_planning

#endif // PLANNING_PLANNER_FRENET_POLYNOMIAL_HPP
