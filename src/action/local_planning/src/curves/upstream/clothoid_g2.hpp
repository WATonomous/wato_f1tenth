#ifndef LOCAL_PLANNING_CURVES_CLOTHOID_G2_HPP
#define LOCAL_PLANNING_CURVES_CLOTHOID_G2_HPP

// Minimal G2 clothoid solver, extracted from Bertolazzi's Clothoids.
//
// The planner needs exactly one operation out of that library: given two
// boundary states, produce the three-clothoid G2 connection between them.  The
// full library carries Dubins curves, biarcs, polylines, AABB trees, curve
// intersection, a generic serialization container, and a formatting library --
// none of which the planner touches, but all of which it paid for in build
// time.  See third_party/VENDORED.md for what was extracted and how to verify
// it against upstream.
//
// The numerics here are upstream's, copied rather than rewritten.  The only
// changes are mechanical: the surrounding class hierarchy is gone, error
// macros became a bool return, and names moved into this namespace.

namespace local_planning
{
namespace clothoid
{

// One clothoid arc: curvature varies linearly with arc length, which is what
// makes it a clothoid.  Everything downstream needs is closed form off these
// six numbers, so no evaluation machinery is exposed.
struct Arc
{
  double x0 = 0.0;       // position at s = 0, m
  double y0 = 0.0;
  double theta0 = 0.0;   // heading at s = 0, rad
  double kappa0 = 0.0;   // curvature at s = 0, 1/m
  double dk = 0.0;       // curvature rate, 1/m^2
  double length = 0.0;   // m

  // theta(s) is the integral of a linear curvature, so it is exactly quadratic.
  double thetaAt(double s) const {return theta0 + s * (kappa0 + 0.5 * dk * s);}
  double kappaAt(double s) const {return kappa0 + dk * s;}
};

// The three arcs of a G2 solution, in order from the start boundary.
struct ThreeArcSolution
{
  Arc arcs[3];
  double totalLength() const {return arcs[0].length + arcs[1].length + arcs[2].length;}
};

// Solves the G2 Hermite problem: a curve leaving (x0, y0) at heading theta0
// with curvature kappa0, arriving at (x1, y1) at theta1 with kappa1, with
// position, heading, and curvature all continuous throughout.
//
// Returns the iteration count on success and -1 if it did not converge.  Both
// the iteration cap and the convergence tolerance are upstream's.
int solveG2(
  double x0, double y0, double theta0, double kappa0,
  double x1, double y1, double theta1, double kappa1,
  ThreeArcSolution & solution);

} // namespace clothoid
} // namespace local_planning

#endif // LOCAL_PLANNING_CURVES_CLOTHOID_G2_HPP
