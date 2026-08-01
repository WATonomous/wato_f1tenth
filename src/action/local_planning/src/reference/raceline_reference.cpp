#include "local_planning/reference/raceline_reference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1e-12;
// Minimum waypoints for a meaningful periodic cubic.
constexpr std::size_t kMinWaypoints = 4;
// Two waypoints closer than this are the same point, not a real segment.
constexpr double kDuplicateWaypointToleranceM = 1e-6;
// Coarse samples per segment before Newton refinement.  Segments are ~0.2 m, so
// this brackets the true foot well inside the basin where Newton converges.
constexpr int kCoarseSamplesPerSegment = 4;
constexpr int kNewtonIterations = 8;

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Solves the cyclic tridiagonal system for a periodic cubic spline's second
// derivatives.  sub/diag/super are the three bands; corner_top_right and
// corner_bottom_left close the loop.  Sherman-Morrison reduces it to two
// ordinary tridiagonal solves.
std::vector<double> solveCyclicTridiagonal(
  const std::vector<double> & sub,
  const std::vector<double> & diag,
  const std::vector<double> & super,
  const std::vector<double> & rhs)
{
  const std::size_t n = diag.size();
  const double corner_top_right = sub[0];
  const double corner_bottom_left = super[n - 1];
  const double gamma = -diag[0];

  std::vector<double> modified_diag = diag;
  modified_diag[0] -= gamma;
  modified_diag[n - 1] -= corner_bottom_left * corner_top_right / gamma;

  // Thomas algorithm, run twice against the same modified matrix.
  auto solve_tridiagonal = [&](const std::vector<double> & b) {
      std::vector<double> c_prime(n, 0.0);
      std::vector<double> d_prime(n, 0.0);
      c_prime[0] = super[0] / modified_diag[0];
      d_prime[0] = b[0] / modified_diag[0];
      for (std::size_t i = 1; i < n; ++i) {
        const double denom = modified_diag[i] - sub[i] * c_prime[i - 1];
        c_prime[i] = super[i] / denom;
        d_prime[i] = (b[i] - sub[i] * d_prime[i - 1]) / denom;
      }
      std::vector<double> x(n, 0.0);
      x[n - 1] = d_prime[n - 1];
      for (std::size_t i = n - 1; i-- > 0; ) {
        x[i] = d_prime[i] - c_prime[i] * x[i + 1];
      }
      return x;
    };

  std::vector<double> u(n, 0.0);
  u[0] = gamma;
  u[n - 1] = corner_bottom_left;

  const std::vector<double> y = solve_tridiagonal(rhs);
  const std::vector<double> z = solve_tridiagonal(u);

  // v = (1, 0, ..., 0, corner_top_right / gamma)
  const double v_dot_y = y[0] + corner_top_right / gamma * y[n - 1];
  const double v_dot_z = z[0] + corner_top_right / gamma * z[n - 1];
  const double factor = v_dot_y / (1.0 + v_dot_z);

  std::vector<double> x(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    x[i] = y[i] - factor * z[i];
  }
  return x;
}

// Second derivatives of the periodic cubic through `values`.  h[i] is the
// length of segment i, which wraps from the last waypoint back to the first.
std::vector<double> secondDerivatives(
  const std::vector<double> & values,
  const std::vector<double> & h)
{
  const std::size_t n = values.size();
  std::vector<double> sub(n, 0.0);
  std::vector<double> diag(n, 2.0);
  std::vector<double> super(n, 0.0);
  std::vector<double> rhs(n, 0.0);

  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t prev = (i + n - 1) % n;
    const std::size_t next = (i + 1) % n;
    const double h_prev = h[prev];
    const double h_curr = h[i];
    const double span = h_prev + h_curr;

    sub[i] = h_prev / span;
    super[i] = h_curr / span;
    rhs[i] = 6.0 / span *
      ((values[next] - values[i]) / h_curr - (values[i] - values[prev]) / h_prev);
  }

  return solveCyclicTridiagonal(sub, diag, super, rhs);
}

} // namespace

bool RacelineReference::setRacingLine(const std::vector<Point> & points)
{
  valid_ = false;
  points_.clear();
  cumulative_s_.clear();
  segment_length_.clear();
  spline_x_.clear();
  spline_y_.clear();
  total_length_m_ = 0.0;

  // Some exporters repeat the first waypoint to close the loop.  The loop is
  // implicit here, and keeping the repeat leaves a zero-length segment that
  // makes the spline system singular.
  points_ = points;
  if (points_.size() >= 2 &&
    std::hypot(
      points_.front().x - points_.back().x,
      points_.front().y - points_.back().y) < kDuplicateWaypointToleranceM)
  {
    points_.pop_back();
  }
  if (points_.size() < kMinWaypoints) {
    return false;
  }

  const std::size_t n = points_.size();
  segment_length_.resize(n);
  cumulative_s_.resize(n);
  cumulative_s_[0] = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t next = (i + 1) % n;
    segment_length_[i] = std::hypot(
      points_[next].x - points_[i].x,
      points_[next].y - points_[i].y);
    if (segment_length_[i] < kEpsilon) {
      return false;
    }
    if (next != 0) {
      cumulative_s_[next] = cumulative_s_[i] + segment_length_[i];
    }
  }
  total_length_m_ = cumulative_s_.back() + segment_length_.back();

  std::vector<double> xs(n);
  std::vector<double> ys(n);
  for (std::size_t i = 0; i < n; ++i) {
    xs[i] = points_[i].x;
    ys[i] = points_[i].y;
  }

  const std::vector<double> mx = secondDerivatives(xs, segment_length_);
  const std::vector<double> my = secondDerivatives(ys, segment_length_);

  auto build = [&](const std::vector<double> & values, const std::vector<double> & m) {
      std::vector<SplineSegment> segments(n);
      for (std::size_t i = 0; i < n; ++i) {
        const std::size_t next = (i + 1) % n;
        const double h = segment_length_[i];
        segments[i].a = values[i];
        segments[i].b = (values[next] - values[i]) / h - h * (2.0 * m[i] + m[next]) / 6.0;
        segments[i].c = m[i] / 2.0;
        segments[i].d = (m[next] - m[i]) / (6.0 * h);
      }
      return segments;
    };

  spline_x_ = build(xs, mx);
  spline_y_ = build(ys, my);
  valid_ = true;
  return true;
}

double RacelineReference::wrapS(double s) const
{
  if (total_length_m_ <= kEpsilon) {
    return 0.0;
  }
  s = std::fmod(s, total_length_m_);
  if (s < 0.0) {
    s += total_length_m_;
  }
  return s;
}

double RacelineReference::deltaS(double from_s, double to_s) const
{
  if (total_length_m_ <= kEpsilon) {
    return 0.0;
  }
  double delta = wrapS(to_s) - wrapS(from_s);
  if (delta > total_length_m_ / 2.0) {
    delta -= total_length_m_;
  } else if (delta < -total_length_m_ / 2.0) {
    delta += total_length_m_;
  }
  return delta;
}

std::size_t RacelineReference::segmentAt(double s_wrapped, double & t) const
{
  // cumulative_s_ is sorted; find the last knot at or before s.
  const auto upper = std::upper_bound(cumulative_s_.begin(), cumulative_s_.end(), s_wrapped);
  std::size_t index = static_cast<std::size_t>(upper - cumulative_s_.begin());
  index = (index == 0) ? 0 : index - 1;
  index = std::min(index, points_.size() - 1);
  t = s_wrapped - cumulative_s_[index];
  return index;
}

ReferenceGeometrySample RacelineReference::sampleAtS(double s) const
{
  ReferenceGeometrySample sample;
  sample.s = s;
  if (!valid_) {
    return sample;
  }

  sample.s_wrapped = wrapS(s);
  double t = 0.0;
  const std::size_t i = segmentAt(sample.s_wrapped, t);
  sample.segment_index = static_cast<int>(i);

  const SplineSegment & sx = spline_x_[i];
  const SplineSegment & sy = spline_y_[i];

  sample.x = sx.a + t * (sx.b + t * (sx.c + t * sx.d));
  sample.y = sy.a + t * (sy.b + t * (sy.c + t * sy.d));

  const double dx = sx.b + t * (2.0 * sx.c + 3.0 * t * sx.d);
  const double dy = sy.b + t * (2.0 * sy.c + 3.0 * t * sy.d);
  const double ddx = 2.0 * sx.c + 6.0 * t * sx.d;
  const double ddy = 2.0 * sy.c + 6.0 * t * sy.d;

  const double speed_sq = dx * dx + dy * dy;
  const double speed = std::sqrt(std::max(speed_sq, kEpsilon));
  sample.tangent_x = dx / speed;
  sample.tangent_y = dy / speed;
  // Left normal, so positive d is left of travel.
  sample.normal_x = -sample.tangent_y;
  sample.normal_y = sample.tangent_x;
  sample.heading = std::atan2(dy, dx);
  sample.curvature = (dx * ddy - dy * ddx) / std::pow(speed_sq, 1.5);

  // Raceline speed interpolates linearly between waypoints:
  const std::size_t next = (i + 1) % points_.size();
  const double alpha = std::clamp(t / segment_length_[i], 0.0, 1.0);
  sample.velocity = points_[i].velocity +
    alpha * (points_[next].velocity - points_[i].velocity);

  return sample;
}

Point RacelineReference::toCartesian(double s, double d) const
{
  const ReferenceGeometrySample sample = sampleAtS(s);
  return Point(
    sample.x + d * sample.normal_x,
    sample.y + d * sample.normal_y,
    sample.velocity);
}

double RacelineReference::refineOnSegment(
  const Point & p,
  std::size_t segment,
  double t_initial) const
{
  const SplineSegment & sx = spline_x_[segment];
  const SplineSegment & sy = spline_y_[segment];
  const double h = segment_length_[segment];

  double t = std::clamp(t_initial, 0.0, h);
  for (int iteration = 0; iteration < kNewtonIterations; ++iteration) {
    const double x = sx.a + t * (sx.b + t * (sx.c + t * sx.d));
    const double y = sy.a + t * (sy.b + t * (sy.c + t * sy.d));
    const double dx = sx.b + t * (2.0 * sx.c + 3.0 * t * sx.d);
    const double dy = sy.b + t * (2.0 * sy.c + 3.0 * t * sy.d);
    const double ddx = 2.0 * sx.c + 6.0 * t * sx.d;
    const double ddy = 2.0 * sy.c + 6.0 * t * sy.d;

    // Foot of the perpendicular: (P(t) - p) . P'(t) = 0.
    const double rx = x - p.x;
    const double ry = y - p.y;
    const double f = rx * dx + ry * dy;
    const double df = dx * dx + dy * dy + rx * ddx + ry * ddy;
    if (std::abs(df) < kEpsilon) {
      break;
    }

    const double step = f / df;
    const double next_t = std::clamp(t - step, 0.0, h);
    if (std::abs(next_t - t) < 1e-10) {
      t = next_t;
      break;
    }
    t = next_t;
  }
  return t;
}

bool RacelineReference::scanSegment(
  const Point & p,
  double heading,
  bool use_tangent_check,
  std::size_t segment,
  Projection & best,
  double & best_dist_sq) const
{
  bool improved = false;

  for (int k = 0; k <= kCoarseSamplesPerSegment; ++k) {
    const double t_guess =
      segment_length_[segment] * static_cast<double>(k) /
      static_cast<double>(kCoarseSamplesPerSegment);
    const double t = refineOnSegment(p, segment, t_guess);
    const double s = cumulative_s_[segment] + t;
    const ReferenceGeometrySample sample = sampleAtS(s);

    const double rx = p.x - sample.x;
    const double ry = p.y - sample.y;
    const double dist_sq = rx * rx + ry * ry;
    if (dist_sq >= best_dist_sq) {
      continue;
    }

    if (use_tangent_check &&
      std::abs(wrapAngle(sample.heading - heading)) >
      projection_config_.tangent_tolerance_rad)
    {
      continue;
    }

    best_dist_sq = dist_sq;
    best.s = wrapS(s);
    best.d = rx * sample.normal_x + ry * sample.normal_y;
    improved = true;
  }

  return improved;
}

Projection RacelineReference::searchWindow(
  const Point & p,
  double heading,
  bool use_tangent_check,
  double seed_s,
  double window_m,
  bool & found) const
{
  found = false;
  Projection best;
  double best_dist_sq = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < points_.size(); ++i) {
    // Skip segments whose whole span lies outside the window.  Comparing at
    // both ends keeps a segment that only partly overlaps.
    const double start_delta = std::abs(deltaS(seed_s, cumulative_s_[i]));
    const double end_delta =
      std::abs(deltaS(seed_s, cumulative_s_[i] + segment_length_[i]));
    if (start_delta > window_m && end_delta > window_m) {
      continue;
    }
    found |= scanSegment(p, heading, use_tangent_check, i, best, best_dist_sq);
  }

  return best;
}

Projection RacelineReference::searchAllSegments(
  const Point & p,
  double heading,
  bool use_tangent_check,
  bool & found) const
{
  found = false;
  Projection best;
  double best_dist_sq = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < points_.size(); ++i) {
    found |= scanSegment(p, heading, use_tangent_check, i, best, best_dist_sq);
  }

  return best;
}

Projection RacelineReference::project(
  const Point & p,
  double heading,
  double seed_s) const
{
  if (!valid_) {
    return {};
  }

  bool found = false;
  Projection result = searchWindow(
    p, heading, true, seed_s, projection_config_.seed_window_m, found);
  if (found) {
    return result;
  }

  // Stale seed: initialization, relocalization, or ego genuinely jumped.
  result = projectGlobal(p, heading, true);
  result.seed_was_stale = true;
  return result;
}

Projection RacelineReference::project(const Point & p, double seed_s) const
{
  if (!valid_) {
    return {};
  }

  bool found = false;
  Projection result = searchWindow(
    p, 0.0, false, seed_s, projection_config_.seed_window_m, found);
  if (found) {
    return result;
  }

  result = projectGlobal(p, 0.0, false);
  result.seed_was_stale = true;
  return result;
}

Projection RacelineReference::projectGlobal(
  const Point & p,
  double heading,
  bool use_tangent_check) const
{
  if (!valid_) {
    return {};
  }

  bool found = false;
  Projection result = searchAllSegments(p, heading, use_tangent_check, found);
  if (!found && use_tangent_check) {
    // Nothing on the whole loop agreed with the heading.  Prefer a nearest-point
    // answer over no answer.
    result = searchAllSegments(p, heading, false, found);
  }
  return result;
}

} // namespace local_planning
