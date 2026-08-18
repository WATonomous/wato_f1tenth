#ifndef LOCAL_PLANNING_CORE_GEOMETRY_HPP
#define LOCAL_PLANNING_CORE_GEOMETRY_HPP

#include <cmath>

namespace local_planning
{

constexpr double kPi = 3.14159265358979323846;
// Occupancy, config near-equality, and other coarse floating comparisons.
constexpr double kGridEps = 1e-6;
// Sample-count / spacing arithmetic (ceil of length/spacing).
constexpr double kSpacingEps = 1e-9;
// Spline segment length, Newton steps, and other high-precision geometry.
constexpr double kSplineEps = 1e-12;

inline double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double shortestAngleDiff(double a, double b)
{
  return wrapAngle(a - b);
}

}  // namespace local_planning

#endif  // LOCAL_PLANNING_CORE_GEOMETRY_HPP
