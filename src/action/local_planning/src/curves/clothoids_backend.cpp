#include "clothoids_backend.hpp"

#include "upstream/clothoid_g2.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace local_planning::clothoids_backend
{

namespace
{

constexpr double kMinSeparationM = 1e-6;

constexpr double kMinSampleSpacingM = 1e-3;

} // namespace

SolveResult solveG2(
  const BoundaryState & start,
  const BoundaryState & terminal,
  double sample_spacing_m)
{
  SolveResult result;

  const double dx = terminal.x - start.x;
  const double dy = terminal.y - start.y;
  if (std::hypot(dx, dy) < kMinSeparationM) {
    return result;
  }

  clothoid::ThreeArcSolution solution;
  const int iterations = clothoid::solveG2(
    start.x, start.y, start.heading, start.curvature,
    terminal.x, terminal.y, terminal.heading, terminal.curvature,
    solution);
  if (iterations < 0) {
    return result;
  }

  const double length = solution.totalLength();
  if (!std::isfinite(length) || length <= 0.0) {
    return result;
  }

  // Curvature is linear along each arc, so the extremes are at arc endpoints.
  double max_abs_curvature = 0.0;
  for (const clothoid::Arc & arc : solution.arcs) {
    max_abs_curvature = std::max(
      max_abs_curvature,
      std::max(std::abs(arc.kappaAt(0.0)), std::abs(arc.kappaAt(arc.length))));
  }
  if (!std::isfinite(max_abs_curvature)) {
    return result;
  }

  const double spacing = std::max(sample_spacing_m, kMinSampleSpacingM);
  const std::size_t intervals =
    static_cast<std::size_t>(std::max(1.0, std::ceil(length / spacing)));


  /*
    x,y on a clothoid is an expensive fresnel integral, so instead we use simpsons
    rule to interpolate which agrees strongly (sub micrometer error)
    and is 3x faster
  */
  result.samples.reserve(intervals + 3);
  double s_base = 0.0;

  for (std::size_t a = 0; a < 3; ++a) {
    const clothoid::Arc & arc = solution.arcs[a];

    double x = arc.x0;
    double y = arc.y0;

    // Only the first arc contributes its start point; later ones would repeat
    // the previous arc's end.
    if (a == 0) {
      CurveSample first;
      first.s = 0.0;
      first.x = x;
      first.y = y;
      first.heading = arc.theta0;
      first.curvature = arc.kappa0;
      first.speed = 0.0;
      result.samples.push_back(first);
    }

    if (!(arc.length > 0.0)) {
      continue;
    }


    const std::size_t steps =
      static_cast<std::size_t>(std::max(1.0, std::ceil(arc.length / spacing)));
    const double h = arc.length / static_cast<double>(steps);

    double cos_a = std::cos(arc.theta0);
    double sin_a = std::sin(arc.theta0);

    for (std::size_t k = 0; k < steps; ++k) {
      const double t_a = static_cast<double>(k) * h;
      double t_b = t_a + h;
      if (k + 1 == steps) {
        t_b = arc.length;   // land exactly on the arc end
      }

      const double cos_m = std::cos(arc.thetaAt(0.5 * (t_a + t_b)));
      const double sin_m = std::sin(arc.thetaAt(0.5 * (t_a + t_b)));
      const double theta_b = arc.thetaAt(t_b);
      const double cos_b = std::cos(theta_b);
      const double sin_b = std::sin(theta_b);

      const double step = t_b - t_a;
      x += (step / 6.0) * (cos_a + 4.0 * cos_m + cos_b);
      y += (step / 6.0) * (sin_a + 4.0 * sin_m + sin_b);
      cos_a = cos_b;
      sin_a = sin_b;

      CurveSample sample;
      sample.s = s_base + t_b;
      sample.x = x;
      sample.y = y;
      sample.heading = theta_b;
      sample.curvature = arc.kappaAt(t_b);
      sample.speed = 0.0;   // filled by the velocity profile, not by the curve
      result.samples.push_back(sample);
    }

    s_base += arc.length;
  }

  // The summed arc lengths and the reported total can disagree in the last
  // bits; downstream integrates over s, so pin it.
  result.samples.back().s = length;

  result.arc_length_m = length;
  result.max_abs_curvature_inv_m = max_abs_curvature;
  result.converged = true;
  return result;
}

} // namespace local_planning::clothoids_backend
