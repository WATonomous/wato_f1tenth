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
// lateralOffsetAt() converges quadratically once the Newton step is divided by
// (1 - kappa*d), so a tight tolerance costs iterations rather than fallbacks and
// there is no reason to trade accuracy away.  Measured on the sim and Mexico
// City racelines: exact to 1e-9 m, under 1% falling through to the windowed
// search.  Do not loosen this to chase a fallback rate -- an undivided step was
// what made 1e-4 unreachable, and loosening to 1e-2 to hide that cost 3.6 cm.
constexpr int kMaxLateralOffsetSteps = 8;
constexpr double kLateralOffsetToleranceM = 1e-6;
// Below this the query point is at the centre of curvature, where the foot of
// the perpendicular is not unique.
constexpr double kMinNewtonDenominator = 1e-3;
// How far the refinement may travel from the hint before declaring it unusable.
// Comfortably covers connect()'s station-interpolation error, which measured
// 0.12 m on the sim raceline and 0.81 m on a deliberately worse one, while
// staying inside the seed window the fallback search would use anyway.
constexpr double kMaxLateralOffsetExcursionM = 1.0;

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

SustainableBounds SustainableBounds::unbounded()
{
  const double infinity = std::numeric_limits<double>::infinity();
  return {infinity, infinity};
}

bool RacelineReference::setRacingLine(const std::vector<Point> & points)
{
  clearTrackWidths();
  valid_ = false;
  points_.clear();
  cumulative_s_.clear();
  segment_length_.clear();
  spline_x_.clear();
  spline_y_.clear();
  total_length_m_ = 0.0;

  // Some exporters repeat the first waypoint to close the loop.  The loop is
  // implicit here, and keeping the repeat leaves a zero-length segment that
  // makes the spline system singular.  Remember that we dropped it so
  // setTrackWidths() can accept the matching, still-closed width vector.
  dropped_closing_waypoint_ = false;
  points_ = points;
  if (points_.size() >= 2 &&
    std::hypot(
      points_.front().x - points_.back().x,
      points_.front().y - points_.back().y) < kDuplicateWaypointToleranceM)
  {
    points_.pop_back();
    dropped_closing_waypoint_ = true;
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

void RacelineReference::clearTrackWidths()
{
  track_widths_valid_ = false;
  width_spacing_m_ = 0.0;
  raw_right_m_.clear();
  raw_left_m_.clear();
}

bool RacelineReference::setTrackWidths(
  const std::vector<TrackWidth> & widths,
  double collision_radius_m,
  double margin_m,
  double requested_spacing_m)
{
  clearTrackWidths();
  // A closed-loop export carries one width per original waypoint, including
  // the repeated closing waypoint that setRacingLine() dropped.  That trailing
  // width duplicates the first and is simply ignored; interpolation below only
  // indexes widths [0, points_.size()).
  const std::size_t expected_count =
    points_.size() + (dropped_closing_waypoint_ ? 1U : 0U);
  if (!valid_ ||
    (widths.size() != points_.size() && widths.size() != expected_count) ||
    requested_spacing_m <= 0.0)
  {
    return false;
  }

  const std::size_t count = std::max<std::size_t>(
    2, static_cast<std::size_t>(std::ceil(total_length_m_ / requested_spacing_m)));
  width_spacing_m_ = total_length_m_ / static_cast<double>(count);
  raw_right_m_.resize(count);
  raw_left_m_.resize(count);
  const double clearance = collision_radius_m + margin_m;

  for (std::size_t i = 0; i < count; ++i) {
    double t = 0.0;
    const std::size_t segment = segmentAt(static_cast<double>(i) * width_spacing_m_, t);
    const std::size_t next = (segment + 1) % points_.size();
    const double alpha = t / segment_length_[segment];
    raw_right_m_[i] = std::max(
      0.0, widths[segment].right_m +
      alpha * (widths[next].right_m - widths[segment].right_m) - clearance);
    raw_left_m_[i] = std::max(
      0.0,
      widths[segment].left_m +
      alpha * (widths[next].left_m - widths[segment].left_m) - clearance);
  }

  track_widths_valid_ = true;
  return true;
}

SustainableBounds RacelineReference::rawBounds(double s) const
{
  if (!track_widths_valid_) {
    return {};
  }
  const std::size_t index = std::min(
    static_cast<std::size_t>(wrapS(s) / width_spacing_m_), raw_left_m_.size() - 1);
  const std::size_t next = (index + 1) % raw_left_m_.size();
  return {
    std::min(raw_right_m_[index], raw_right_m_[next]),
    std::min(raw_left_m_[index], raw_left_m_[next])};
}

WidthLookupSample RacelineReference::widthSample(std::size_t index) const
{
  if (!track_widths_valid_ || index >= raw_left_m_.size()) {
    return {};
  }
  return {
    static_cast<double>(index) * width_spacing_m_,
    {raw_right_m_[index], raw_left_m_[index]}};
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

double RacelineReference::lateralOffsetAt(
  const Point & p, double s_hint, bool * converged, double * refined_s) const
{
  if (converged != nullptr) {
    *converged = false;
  }
  if (!valid_) {
    return 0.0;
  }

  // Newton on arc length, seeking the station where the offset vector is
  // orthogonal to the tangent.  For f(s) = (p - ref(s)) . T(s), the Frenet
  // relations give f'(s) = -(1 - kappa * d), so the step is along / (1 - kappa*d).
  //
  // That denominator is the whole reason this is not simply "subtract the
  // tangential residual".  Approaching the centre of curvature, kappa*d -> 1 and
  // the residual goes to zero while the station error does not: at kappa 1.74
  // and d 0.55 it shrinks by 23x, so an undivided step stalls far from the foot
  // and reports success.  Dividing recovers true quadratic convergence.
  double s = s_hint;
  ReferenceGeometrySample reference = sampleAtS(s);
  double dx = p.x - reference.x;
  double dy = p.y - reference.y;

  for (int step = 0; step < kMaxLateralOffsetSteps; ++step) {
    const double along = dx * reference.tangent_x + dy * reference.tangent_y;
    if (std::abs(along) <= kLateralOffsetToleranceM) {
      if (converged != nullptr) {
        *converged = true;
      }
      if (refined_s != nullptr) {
        *refined_s = s;
      }
      break;
    }
    const double offset = dx * reference.normal_x + dy * reference.normal_y;
    // At or past the centre of curvature the foot is not unique and Newton has
    // no useful direction.  Clamping keeps the step finite; the excursion test
    // below then rejects it rather than letting it wander.
    double denominator = 1.0 - reference.curvature * offset;
    if (std::abs(denominator) < kMinNewtonDenominator) {
      denominator = std::copysign(kMinNewtonDenominator, denominator);
    }

    // A small denominator makes the step enormous, and on a closed loop an
    // unbounded step lands in a different part of the track, where Newton
    // happily converges to the wrong foot and reports success -- measured 9.7 m
    // of error before this bound existed.  The contract is "refine a nearby
    // station", so leaving the neighbourhood means the hint was unusable, not
    // that the answer is far away.  Say so and let the caller search.
    const double step_s = std::clamp(
      along / denominator, -kMaxLateralOffsetExcursionM, kMaxLateralOffsetExcursionM);
    s = wrapS(s + step_s);
    if (std::abs(deltaS(s_hint, s)) > kMaxLateralOffsetExcursionM) {
      break;
    }
    reference = sampleAtS(s);
    dx = p.x - reference.x;
    dy = p.y - reference.y;
  }

  return dx * reference.normal_x + dy * reference.normal_y;
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

Projection RacelineReference::searchArc(
  const Point & p,
  double heading,
  bool use_tangent_check,
  double start_s,
  double length_m,
  bool & found) const
{
  found = false;
  Projection best;
  double best_dist_sq = std::numeric_limits<double>::max();
  if (points_.empty() || length_m <= 0.0) {
    return best;
  }

  length_m = std::min(length_m, total_length_m_);
  double t = 0.0;
  std::size_t i = segmentAt(wrapS(start_s), t);
  const std::size_t n = points_.size();
  double covered = 0.0;
  for (std::size_t k = 0; k < n; ++k) {
    found |= scanSegment(p, heading, use_tangent_check, i, best, best_dist_sq);
    covered += (k == 0) ? (segment_length_[i] - t) : segment_length_[i];
    if (covered >= length_m - kEpsilon) {
      break;
    }
    i = (i + 1) % n;
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

  const double window = projection_config_.seed_window_m;
  bool found = false;
  Projection result = searchArc(
    p, heading, true, seed_s - window, 2.0 * window, found);
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

  const double window = projection_config_.seed_window_m;
  bool found = false;
  Projection result = searchArc(
    p, 0.0, false, seed_s - window, 2.0 * window, found);
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
