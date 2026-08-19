#include "local_planning/reference/raceline_reference.hpp"

#include "local_planning/core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>


/*
  Make sure the way we treat loops is the same like if
  we duplicate the start and end point
*/
namespace local_planning
{
namespace
{

constexpr double kEpsilon = kSplineEps;
constexpr std::size_t kMinWaypoints = 4; //for a meaningful spline
constexpr double kDuplicateWaypointToleranceM = 1e-6;

constexpr int kCoarseSamplesPerSegment = 4; //samples per segment before newton refinement
constexpr int kNewtonIterations = 8;
constexpr double kTangentCheckDisabled = kPi;
constexpr double kMaxTangentToleranceRad = kPi / 2.0; //this should lowkey be smaller cant even lie twin
constexpr double kMinTangentToleranceRad = 0.05;

// Solves the cyclic tridiagonal system for a periodic cubic spline's second derivatives. 
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

// Second derivatives of the periodic cubic through `values`. 
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

//when you get a new raceline
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

 //we need to drop the duplicate waypoint to make the system full rank
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

/*
build a lookup table for left and right space along the raceline
*/
bool RacelineReference::setTrackWidths(
  const std::vector<TrackWidth> & widths,
  double requested_spacing_m)
{
  clearTrackWidths();

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

  for (std::size_t i = 0; i < count; ++i) {
    double t = 0.0;
    const std::size_t segment = segmentAt(static_cast<double>(i) * width_spacing_m_, t);
    const std::size_t next = (segment + 1) % points_.size();
    const double alpha = t / segment_length_[segment];
    raw_right_m_[i] = std::max(
      0.0, widths[segment].right_m +
      alpha * (widths[next].right_m - widths[segment].right_m));
    raw_left_m_[i] = std::max(
      0.0,
      widths[segment].left_m +
      alpha * (widths[next].left_m - widths[segment].left_m));
  }

  track_widths_valid_ = true;
  return true;
}
/*
get the spacing for a point you cant currently see
the idea is that you only have one opponent so everywhere else that is supposed to be
free is free 
*/
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

//get the start of the segment in s value and the t value is the distance along the segment
std::size_t RacelineReference::segmentAt(double s_wrapped, double & t) const
{
  // cumulative_s_ is sorted find the last knot at or before s.
  const auto upper = std::upper_bound(cumulative_s_.begin(), cumulative_s_.end(), s_wrapped);
  std::size_t index = static_cast<std::size_t>(upper - cumulative_s_.begin());
  index = (index == 0) ? 0 : index - 1;
  index = std::min(index, points_.size() - 1);
  t = s_wrapped - cumulative_s_[index];
  return index;
}

double RacelineReference::velocityOnSegment(std::size_t i, double t) const
{
  const std::size_t next = (i + 1) % points_.size();
  const double alpha = std::clamp(t / segment_length_[i], 0.0, 1.0);
  return points_[i].velocity + alpha * (points_[next].velocity - points_[i].velocity);
}


/*
  calculate a bunch of info about all the geometric properties at point s
*/
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
  sample.parameter_speed = speed;
  sample.tangent_x = dx / speed;
  sample.tangent_y = dy / speed;
  // Left normal, so positive d is left of travel.
  sample.normal_x = -sample.tangent_y;
  sample.normal_y = sample.tangent_x;
  sample.heading = std::atan2(dy, dx);
  const double cross = dx * ddy - dy * ddx;
  sample.curvature = cross / (speed_sq * speed);

  
  const double dddx = 6.0 * sx.d;
  const double dddy = 6.0 * sy.d;
  const double cross_derivative = dx * dddy - dy * dddx;   // the ddx*ddy terms cancel
  const double dot = dx * ddx + dy * ddy;
  sample.curvature_derivative =
    (cross_derivative * speed_sq - 3.0 * cross * dot) /
    (speed_sq * speed_sq * speed_sq);
  sample.velocity = velocityOnSegment(i, t);

  return sample;
}

double RacelineReference::velocityAtS(double s) const
{
  if (!valid_) {
    return 0.0;
  }

  double t = 0.0;
  const std::size_t i = segmentAt(wrapS(s), t);
  return velocityOnSegment(i, t);
}

Point RacelineReference::toCartesian(double s, double d) const
{
  const ReferenceGeometrySample sample = sampleAtS(s);
  return Point(
    sample.x + d * sample.normal_x,
    sample.y + d * sample.normal_y,
    sample.velocity);
}
/*
  once we lock in on a segment we can refine to find the best point
  along it
*/
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
    //clamp steps to prevent diabolical jumps
    const double next_t = std::clamp(t - step, 0.0, h);
    if (std::abs(next_t - t) < 1e-5) { 
      t = next_t;
      break;
    }
    t = next_t;
  }
  return t;
}


//similiar to above but consider angle too
bool RacelineReference::scanSegment(
  const Point & p,
  double heading,
  double tolerance_rad,
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

    if (std::abs(shortestAngleDiff(sample.heading, heading)) > tolerance_rad) {
      continue;
    }

    best_dist_sq = dist_sq;
    best.s = wrapS(s);
    best.d = rx * sample.normal_x + ry * sample.normal_y;
    improved = true;
  }

  return improved;
}

//same as above but over multiple segments
Projection RacelineReference::searchArc(
  const Point & p,
  double heading,
  double tolerance_rad,
  double start_s,
  double length_m,
  bool & found,
  double & best_dist_sq) const
{
  found = false;
  Projection best;
  if (points_.empty() || length_m <= 0.0) {
    return best;
  }

  length_m = std::min(length_m, total_length_m_);
  double t = 0.0;
  std::size_t i = segmentAt(wrapS(start_s), t);
  const std::size_t n = points_.size();
  double covered = 0.0;
  for (std::size_t k = 0; k < n; ++k) {
    found |= scanSegment(p, heading, tolerance_rad, i, best, best_dist_sq);
    covered += (k == 0) ? (segment_length_[i] - t) : segment_length_[i];
    if (covered >= length_m - kEpsilon) {
      break;
    }
    i = (i + 1) % n;
  }
  return best;
}

//globla search
Projection RacelineReference::searchAllSegments(
  const Point & p,
  double heading,
  double tolerance_rad,
  bool & found) const
{
  found = false;
  Projection best;
  double best_dist_sq = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < points_.size(); ++i) {
    found |= scanSegment(p, heading, tolerance_rad, i, best, best_dist_sq);
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
  const double start_s = seed_s - window;
  const double length_m = 2.0 * window;
  const double gate_sq = projection_config_.max_plausible_offset_m *
    projection_config_.max_plausible_offset_m;

  // Relax the heading check for sliding or aggressive cornering, but keep a
  // distance gate to reject stale seed windows. Never disable the heading
  // check; failed local searches fall back to the global search.
  const double base_tolerance = std::clamp(
    projection_config_.tangent_tolerance_rad,
    kMinTangentToleranceRad,
    kMaxTangentToleranceRad);
  bool relaxed = false;
  for (double tolerance = base_tolerance; ;
    tolerance = std::min(2.0 * tolerance, kMaxTangentToleranceRad)) //this is a little chopped
    //we might want kMaxTangentToleranceRad to be smaller pi/2 is obviously pushing it
  {
    bool found = false;
    double best_dist_sq = std::numeric_limits<double>::max();
    Projection result =
      searchArc(p, heading, tolerance, start_s, length_m, found, best_dist_sq);
    if (found && best_dist_sq <= gate_sq) {
      result.heading_check_relaxed = relaxed;
      return result;
    }
    if (tolerance >= kMaxTangentToleranceRad) {
      break;
    }
    relaxed = true;
  }

  // The seed window is stale or invalid; use the global search.
  Projection result = projectGlobal(p, heading, base_tolerance);
  result.seed_was_stale = true;
  return result;
}

Projection RacelineReference::project(const Point & p, double seed_s) const
{
  if (!valid_) {
    return {};
  }

  // No heading to check and no offset worth gating: an occupied cell is
  // legitimately metres off the reference, so the window is the only
  // disambiguator and there is nothing for the ladder above to escalate.
  const double window = projection_config_.seed_window_m;
  bool found = false;
  double best_dist_sq = std::numeric_limits<double>::max();
  Projection result = searchArc(
    p, 0.0, kTangentCheckDisabled, seed_s - window, 2.0 * window, found, best_dist_sq);
  if (found) {
    return result;
  }

  result = projectGlobal(p, 0.0, kTangentCheckDisabled);
  result.seed_was_stale = true;
  return result;
}

Projection RacelineReference::projectGlobal(
  const Point & p,
  double heading,
  double tolerance_rad) const
{
  if (!valid_) {
    return {};
  }

  bool found = false;
  Projection result = searchAllSegments(p, heading, tolerance_rad, found);
  if (!found && tolerance_rad < kPi) {
    // Nothing on the whole loop agreed with the heading.  Prefer a nearest-point
    // answer over no answer.
    result = searchAllSegments(p, heading, kTangentCheckDisabled, found);
    result.heading_check_relaxed = true;
  }
  return result;
}

} // namespace local_planning
