#include "planning/planner/spline_refinement.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1e-6;

// C2 natural cubic spline of a single coordinate against a parameter u.
// The second derivatives are pinned to zero at both ends (natural); the
// interior second derivatives come from a tridiagonal solve (Thomas).
struct NaturalCubicSpline1D
{
  std::vector<double> u;
  std::vector<double> f;
  std::vector<double> m;   // second derivative at each knot

  bool build(const std::vector<double> & u_in, const std::vector<double> & f_in)
  {
    if (u_in.size() != f_in.size() || u_in.size() < 2) {
      return false;
    }
    u = u_in;
    f = f_in;
    const std::size_t n = u.size();
    m.assign(n, 0.0);
    if (n < 3) {
      return true;   // two knots: straight line, m stays zero
    }

    // Interior unknowns m[1 .. n-2] form a tridiagonal system.
    const std::size_t interior = n - 2;
    std::vector<double> lower(interior, 0.0);
    std::vector<double> diag(interior, 0.0);
    std::vector<double> upper(interior, 0.0);
    std::vector<double> rhs(interior, 0.0);
    for (std::size_t k = 0; k < interior; ++k) {
      const std::size_t i = k + 1;
      const double h_prev = u[i] - u[i - 1];
      const double h_next = u[i + 1] - u[i];
      if (h_prev <= kEpsilon || h_next <= kEpsilon) {
        return false;
      }
      lower[k] = h_prev;
      diag[k] = 2.0 * (h_prev + h_next);
      upper[k] = h_next;
      rhs[k] = 6.0 * ((f[i + 1] - f[i]) / h_next - (f[i] - f[i - 1]) / h_prev);
    }

    // Thomas algorithm (m[0] and m[n-1] remain zero from the natural BCs).
    for (std::size_t k = 1; k < interior; ++k) {
      if (std::abs(diag[k - 1]) <= kEpsilon) {
        return false;
      }
      const double factor = lower[k] / diag[k - 1];
      diag[k] -= factor * upper[k - 1];
      rhs[k] -= factor * rhs[k - 1];
    }
    if (std::abs(diag[interior - 1]) <= kEpsilon) {
      return false;
    }
    m[interior] = rhs[interior - 1] / diag[interior - 1];
    for (std::size_t k = interior - 1; k-- > 0; ) {
      m[k + 1] = (rhs[k] - upper[k] * m[k + 2]) / diag[k];
    }
    return true;
  }

  std::size_t locate(double uu) const
  {
    // Return segment index j with u[j] <= uu <= u[j+1] (clamped to range).
    if (uu <= u.front()) {
      return 0;
    }
    if (uu >= u.back()) {
      return u.size() - 2;
    }
    std::size_t j = 0;
    while (j + 2 < u.size() && uu > u[j + 1]) {
      ++j;
    }
    return j;
  }

  void sample(double uu, double & value, double & deriv, double & second) const
  {
    const std::size_t j = locate(uu);
    const double h = u[j + 1] - u[j];
    const double left = u[j + 1] - uu;
    const double right = uu - u[j];
    value =
      m[j] * left * left * left / (6.0 * h) +
      m[j + 1] * right * right * right / (6.0 * h) +
      (f[j] / h - m[j] * h / 6.0) * left +
      (f[j + 1] / h - m[j + 1] * h / 6.0) * right;
    deriv =
      -m[j] * left * left / (2.0 * h) +
      m[j + 1] * right * right / (2.0 * h) -
      (f[j] / h - m[j] * h / 6.0) +
      (f[j + 1] / h - m[j + 1] * h / 6.0);
    second = (m[j] * left + m[j + 1] * right) / h;
  }
};

// Quintic segment f(w), w in [0, h], clamped at both ends: at w = 0 the value,
// first and second derivatives are the vehicle boundary; at w = h they match
// the interior cubic spline (value, slope, zero second derivative).  Six
// coefficients absorb the extra start curvature constraint that a cubic spline
// alone cannot accept without over-constraining the terminal end.
struct Quintic1D
{
  double c[6]{};
  double h = 0.0;

  void build(
    double f0, double d0, double s0,
    double f1, double d1, double s1,
    double length)
  {
    h = length;
    c[0] = f0;
    c[1] = d0;
    c[2] = 0.5 * s0;
    const double r = f1 - c[0] - c[1] * h - c[2] * h * h;
    const double p = (d1 - c[1] - 2.0 * c[2] * h) * h;
    const double q = (s1 - 2.0 * c[2]) * h * h;
    const double a = 10.0 * r - 4.0 * p + 0.5 * q;
    const double b = 7.0 * p - 15.0 * r - q;
    const double cc = 0.5 * q + 6.0 * r - 3.0 * p;
    c[3] = a / (h * h * h);
    c[4] = b / (h * h * h * h);
    c[5] = cc / (h * h * h * h * h);
  }

  void sample(double w, double & value, double & deriv, double & second) const
  {
    value = c[0] + w * (c[1] + w * (c[2] + w * (c[3] + w * (c[4] + w * c[5]))));
    deriv = c[1] + w * (2.0 * c[2] + w * (3.0 * c[3] + w * (4.0 * c[4] + w * 5.0 * c[5])));
    second = 2.0 * c[2] + w * (6.0 * c[3] + w * (12.0 * c[4] + w * 20.0 * c[5]));
  }
};

// Cartesian knots for the spline: measured pose + crude lattice anchors.
std::vector<Point> buildKnots(
  const std::vector<FrenetPoint> & anchors,
  const Odometry & odom,
  const FrenetConverter & frenet_converter)
{
  std::vector<Point> knots;
  knots.reserve(anchors.size());
  knots.push_back(odom.position);
  for (std::size_t i = 1; i < anchors.size(); ++i) {
    knots.push_back(frenet_converter.frenetToCartesian(anchors[i]));
  }

  // Drop near-duplicate consecutive knots so chord-length parameterization
  // never divides by a zero segment length.
  std::vector<Point> cleaned;
  cleaned.reserve(knots.size());
  for (const Point & k : knots) {
    if (cleaned.empty() ||
      std::hypot(k.x - cleaned.back().x, k.y - cleaned.back().y) > kEpsilon)
    {
      cleaned.push_back(k);
    }
  }
  return cleaned;
}

} // namespace

SplineRefinementResult refineSplineGeometry(
  const std::vector<FrenetPoint> & anchors,
  const Odometry & odom,
  const FrenetConverter & frenet_converter,
  const LocalFrenetPlannerConfig & config)
{
  SplineRefinementResult result;
  if (anchors.size() < 3) {
    return result;
  }

  const double sample_spacing =
    config.spline_sample_spacing_m > kEpsilon ?
    config.spline_sample_spacing_m : config.sample_spacing_m;
  if (sample_spacing <= kEpsilon) {
    return result;
  }

  const std::vector<Point> knots =
    buildKnots(anchors, odom, frenet_converter);
  if (knots.size() < 3) {
    return result;
  }

  // Chord-length parameter u for every knot (u[0] = 0).
  std::vector<double> u(knots.size(), 0.0);
  for (std::size_t i = 1; i < knots.size(); ++i) {
    u[i] = u[i - 1] +
      std::hypot(knots[i].x - knots[i - 1].x, knots[i].y - knots[i - 1].y);
  }
  const double u_start = u[1];
  const double u_end = u.back();
  if (u_start <= kEpsilon || u_end - u_start <= kEpsilon) {
    return result;
  }

  // Interior C2 natural cubic spline over knots[1 .. n] (excludes the start
  // knot, which the quintic owns).
  std::vector<double> interior_u(u.begin() + 1, u.end());
  std::vector<double> interior_x;
  std::vector<double> interior_y;
  interior_x.reserve(interior_u.size());
  interior_y.reserve(interior_u.size());
  for (std::size_t i = 1; i < knots.size(); ++i) {
    interior_x.push_back(knots[i].x);
    interior_y.push_back(knots[i].y);
  }
  NaturalCubicSpline1D spline_x;
  NaturalCubicSpline1D spline_y;
  if (!spline_x.build(interior_u, interior_x) || !spline_y.build(interior_u, interior_y)) {
    return result;
  }

  // Start clamp: unit-speed tangent from odometry heading; curvature from
  // odom.steering_angle (fresh /drive/autonomy, else 0). For a unit-speed curve
  // r''(u) = kappa * normal.
  const double theta = odom.heading;
  const double kappa0 = config.wheelbase_m > kEpsilon ?
    std::tan(odom.steering_angle) / config.wheelbase_m :
    0.0;
  const double d0x = std::cos(theta);
  const double d0y = std::sin(theta);
  const double s0x = -kappa0 * std::sin(theta);
  const double s0y = kappa0 * std::cos(theta);

  // Slope of the interior spline at its left end (u_start); its second
  // derivative there is zero by the natural boundary condition.
  double d1x = 0.0;
  double d1y = 0.0;
  double ignore = 0.0;
  spline_x.sample(u_start, ignore, d1x, ignore);
  spline_y.sample(u_start, ignore, d1y, ignore);

  Quintic1D quintic_x;
  Quintic1D quintic_y;
  quintic_x.build(knots[0].x, d0x, s0x, knots[1].x, d1x, 0.0, u_start);
  quintic_y.build(knots[0].y, d0y, s0y, knots[1].y, d1y, 0.0, u_start);

  // Dense sampling: quintic over [0, u_start], interior spline over
  // [u_start, u_end]. Exact parametric derivatives supply heading (validation /
  // cusp guard) and analytic curvature (velocity limits on success).
  struct DenseSample
  {
    double x;
    double y;
    double heading;
    double curvature;
  };
  std::vector<DenseSample> dense;

  auto push_sample = [&](double dx, double dy, double ddx, double ddy, double px, double py) {
      const double speed_sq = dx * dx + dy * dy;
      double curvature = 0.0;
      if (speed_sq > kEpsilon) {
        curvature = (dx * ddy - dy * ddx) / std::pow(speed_sq, 1.5);
      }
      dense.push_back({px, py, std::atan2(dy, dx), curvature});
    };

  const int quintic_samples =
    std::max(2, static_cast<int>(std::ceil(u_start / sample_spacing)) + 1);
  for (int k = 0; k < quintic_samples; ++k) {
    const double w = u_start * static_cast<double>(k) /
      static_cast<double>(quintic_samples - 1);
    double x = 0.0, y = 0.0, dx = 0.0, dy = 0.0, ddx = 0.0, ddy = 0.0;
    quintic_x.sample(w, x, dx, ddx);
    quintic_y.sample(w, y, dy, ddy);
    push_sample(dx, dy, ddx, ddy, x, y);
  }

  const double interior_length = u_end - u_start;
  const int interior_samples =
    std::max(2, static_cast<int>(std::ceil(interior_length / sample_spacing)) + 1);
  for (int k = 1; k < interior_samples; ++k) {
    const double uu = u_start + interior_length * static_cast<double>(k) /
      static_cast<double>(interior_samples - 1);
    double x = 0.0, y = 0.0, dx = 0.0, dy = 0.0, ddx = 0.0, ddy = 0.0;
    spline_x.sample(uu, x, dx, ddx);
    spline_y.sample(uu, y, dy, ddy);
    push_sample(dx, dy, ddx, ddy, x, y);
  }

  if (dense.size() < 2) {
    return result;
  }

  // Spline-only forward-progress / cusp guard: the step must not fold back on
  // the sample's own tangent direction. Shared collision / heading validation
  // and velocity assignment happen in path_processing.
  result.path.reserve(dense.size());
  result.headings.reserve(dense.size());
  result.curvatures.reserve(dense.size());
  for (std::size_t i = 0; i < dense.size(); ++i) {
    const DenseSample & sample = dense[i];
    if (i > 0) {
      const double step_x = sample.x - dense[i - 1].x;
      const double step_y = sample.y - dense[i - 1].y;
      if (step_x * std::cos(sample.heading) + step_y * std::sin(sample.heading) < 0.0) {
        result.path.clear();
        result.headings.clear();
        result.curvatures.clear();
        return result;
      }
    }

    result.path.emplace_back(sample.x, sample.y);
    result.headings.push_back(sample.heading);
    result.curvatures.push_back(sample.curvature);
  }

  result.success = true;
  return result;
}

} // namespace local_planning
