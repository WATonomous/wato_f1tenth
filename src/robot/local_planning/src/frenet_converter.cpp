#include "planning/frenet_converter.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace local_planning
{

FrenetConverter::FrenetConverter()
: total_length_(0.0) {}

/*
    to get the total distance we linearly connect all the points on the racing line
    using a cubic to connect groups of 3 points would work better and give a more
    reasonable approximation
    should be pretty easy numerical integration
    TODO: do that at some point :)
*/
void FrenetConverter::setRacingLine(const std::vector<Point> & racing_line)
{
  racing_line_ = racing_line;
  distance_prefix_sum_.clear();
  waypoint_curvatures_.clear();
  total_length_ = 0.0;

  if (racing_line_.size() < 2) {
    return;
  }

  //distance_prefix_sum_[i] =
  //linear distance of (0 to 1) + (1 to 2) + ... + (n-1 to n)
  distance_prefix_sum_.resize(racing_line_.size());
  distance_prefix_sum_[0] = 0.0;
  for (int i = 1; i < static_cast<int>(racing_line_.size()); i++) {
    //get the differential from i-1 to i
    double dx = racing_line_[i].x - racing_line_[i - 1].x;
    double dy = racing_line_[i].y - racing_line_[i - 1].y;
    distance_prefix_sum_[i] = distance_prefix_sum_[i - 1] + std::hypot(dx, dy);
  }

  /*
  to get the total length we need to close the last part of the loop
  the distance from point n back to point 1
  */
  double dx = racing_line_.front().x - racing_line_.back().x;
  double dy = racing_line_.front().y - racing_line_.back().y;
  total_length_ = distance_prefix_sum_.back() + std::hypot(dx, dy);

  // precompute curvature at each waypoint (closed loop)
  // theta[i] = heading of segment i to (i+1)%n
  int n = static_cast<int>(racing_line_.size());
  std::vector<double> seg_headings(n);
  for (int i = 0; i < n; ++i) {
    int j = (i + 1) % n;
    double sdx = racing_line_[j].x - racing_line_[i].x;
    double sdy = racing_line_[j].y - racing_line_[i].y;
    seg_headings[i] = std::atan2(sdy, sdx);
  }

  waypoint_curvatures_.resize(n);
  for (int i = 0; i < n; ++i) {
    int prev = (i - 1 + n) % n;

    double dtheta = seg_headings[i] - seg_headings[prev];
    dtheta = std::atan2(std::sin(dtheta), std::cos(dtheta));     // wrap to [-pi, pi]

    // length of the segment ending at waypoint i (prev to i)
    double len_prev;
    if (prev == n - 1) {
      len_prev = total_length_ - distance_prefix_sum_[prev];
    } else {
      len_prev = distance_prefix_sum_[prev + 1] - distance_prefix_sum_[prev];
    }

    // length of the segment starting at waypoint i (i ti next)
    double len_curr;
    if (i == n - 1) {
      len_curr = total_length_ - distance_prefix_sum_[i];
    } else {
      len_curr = distance_prefix_sum_[i + 1] - distance_prefix_sum_[i];
    }

    double avg_len = (len_prev + len_curr) * 0.5;
    if (avg_len > 1e-9) {     //maybe this should be made to like 1e-3 or something safer ngl
      waypoint_curvatures_[i] = dtheta / avg_len;
    } else {
      waypoint_curvatures_[i] = 0.0;
    }
  }
}

double FrenetConverter::getTotalLength() const
{
  return total_length_;
}

double FrenetConverter::wrapS(double s) const
{
  if (total_length_ <= 1e-12) {
    return 0.0;
  }
  // bring s back into the interval [0, total length)
  s = std::fmod(s, total_length_);
  if (s < 0.0) {
    s += total_length_;
  }
  return s;
}

// Returns the index of the racing line waypoint closest to p.
// todo: use linear interpolation to improve accuracy
// todo part 2: use a higher degree polynomial interpolation
int FrenetConverter::findClosestPoint(const Point & p) const
{
  if (racing_line_.empty()) {
    return 0;
  }

  int best_idx = 0;
  double best_dist = std::numeric_limits<double>::max();

  for (int i = 0; i < static_cast<int>(racing_line_.size()); i++) {
    double dist = std::hypot(p.x - racing_line_[i].x, p.y - racing_line_[i].y);
    if (dist < best_dist) {
      best_dist = dist;
      best_idx = i;
    }
  }

  return best_idx;
}


FrenetPoint FrenetConverter::cartesianToFrenet(const Point & p) const
{
  if (racing_line_.size() < 2 || total_length_ <= 1e-12) {
    return {0.0, 0.0};
  }

  const int n = static_cast<int>(racing_line_.size());
  double best_dist_sq = std::numeric_limits<double>::max();
  double best_s = 0.0;
  double best_d = 0.0;

  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const double ax = racing_line_[i].x;
    const double ay = racing_line_[i].y;
    const double abx = racing_line_[j].x - ax;
    const double aby = racing_line_[j].y - ay;
    const double seg_len_sq = abx * abx + aby * aby;
    if (seg_len_sq < 1e-12) {
      continue;
    }

    const double apx = p.x - ax;
    const double apy = p.y - ay;
    const double t = std::clamp((apx * abx + apy * aby) / seg_len_sq, 0.0, 1.0);
    const double proj_x = ax + t * abx;
    const double proj_y = ay + t * aby;
    const double dx = p.x - proj_x;
    const double dy = p.y - proj_y;
    const double dist_sq = dx * dx + dy * dy;

    if (dist_sq < best_dist_sq) {
      const double seg_len = std::sqrt(seg_len_sq);
      best_dist_sq = dist_sq;
      best_s = distance_prefix_sum_[i] + t * seg_len;
      best_d = dx * (-aby / seg_len) + dy * (abx / seg_len);
    }
  }

  return {wrapS(best_s), best_d};
}

ReferenceGeometrySample FrenetConverter::sampleAtS(double s) const
{
  ReferenceGeometrySample sample;
  sample.s = s;
  if (racing_line_.size() < 2 || total_length_ <= 1e-12) {
    return sample;
  }

  const int n = static_cast<int>(racing_line_.size());
  sample.s_wrapped = wrapS(s);

  auto upper = std::upper_bound(
    distance_prefix_sum_.begin(), distance_prefix_sum_.end(), sample.s_wrapped);
  int i = static_cast<int>(std::distance(distance_prefix_sum_.begin(), upper)) - 1;
  i = std::clamp(i, 0, n - 1);
  const int j = (i + 1) % n;
  sample.segment_index = i;

  const double seg_start = distance_prefix_sum_[i];
  const double abx = racing_line_[j].x - racing_line_[i].x;
  const double aby = racing_line_[j].y - racing_line_[i].y;
  const double seg_len = (j == 0) ?
    total_length_ - distance_prefix_sum_[i] :
    distance_prefix_sum_[j] - distance_prefix_sum_[i];

  double t = 0.0;
  if (seg_len > 1e-12) {
    t = std::clamp((sample.s_wrapped - seg_start) / seg_len, 0.0, 1.0);
    sample.tangent_x = abx / seg_len;
    sample.tangent_y = aby / seg_len;
    sample.normal_x = -sample.tangent_y;
    sample.normal_y = sample.tangent_x;
  }

  sample.x = racing_line_[i].x + t * abx;
  sample.y = racing_line_[i].y + t * aby;
  sample.heading = std::atan2(aby, abx);
  sample.velocity = racing_line_[i].velocity + t *
    (racing_line_[j].velocity - racing_line_[i].velocity);
  if (!waypoint_curvatures_.empty()) {
    sample.curvature = waypoint_curvatures_[i] + t *
      (waypoint_curvatures_[j] - waypoint_curvatures_[i]);
  }
  return sample;
}

Point FrenetConverter::frenetToCartesian(const FrenetPoint & fp) const
{
  if (racing_line_.size() < 2 || total_length_ <= 1e-12) {
    return {0.0, 0.0, 0.0};
  }

  int n = static_cast<int>(racing_line_.size());
  double s = wrapS(fp.s);

  auto upper = std::upper_bound(distance_prefix_sum_.begin(), distance_prefix_sum_.end(), s);
  int i = static_cast<int>(std::distance(distance_prefix_sum_.begin(), upper)) - 1;
  i = std::clamp(i, 0, n - 1);
  int j = (i + 1) % n;

  // how far along the segment ia s
  double seg_start = distance_prefix_sum_[i];
  double abx = racing_line_[j].x - racing_line_[i].x;
  double aby = racing_line_[j].y - racing_line_[i].y;
  double seg_len = std::hypot(abx, aby);

  double t = 0.0;
  double nx = 0.0;
  double ny = 0.0;
  if (seg_len > 1e-12) {
    t = std::clamp((s - seg_start) / seg_len, 0.0, 1.0);

    //left normal is (-sinx, cosx)
    nx = -aby / seg_len;
    ny = abx / seg_len;
  }

  // point on racing line at s which is offset laterally by d
  double cx = racing_line_[i].x + t * abx;
  double cy = racing_line_[i].y + t * aby;

  // target speed from the racing line at this arc length position
  double velocity = racing_line_[i].velocity + t *
    (racing_line_[j].velocity - racing_line_[i].velocity);

  return {cx + fp.d * nx, cy + fp.d * ny, velocity};
}

Point FrenetConverter::frenetToCartesian(const ReferenceGeometrySample & ref, double d)
{
  return {ref.x + d * ref.normal_x, ref.y + d * ref.normal_y, ref.velocity};
}

double FrenetConverter::getRacingLineHeading(double s) const
{
  return sampleAtS(s).heading;
}

double FrenetConverter::getRacingLineVelocity(double s) const
{
  return sampleAtS(s).velocity;
}

double FrenetConverter::getRacingLineCurvature(double s) const
{
  return sampleAtS(s).curvature;
}

void FrenetConverter::fillReferenceGeometryTable(
  const std::vector<double> & s_values,
  std::vector<ReferenceGeometrySample> & out) const
{
  const std::size_t needed = s_values.size();
  out.resize(needed);

  for (std::size_t i = 0; i < needed; ++i) {
    out[i] = sampleAtS(s_values[i]);
  }
}

void FrenetConverter::fillUniformReferenceGeometryTable(
  double s_start,
  int layer_count,
  double layer_spacing_m,
  int sample_count,
  std::vector<ReferenceGeometrySample> & out) const
{
  if (layer_count <= 0 || sample_count < 2 || layer_spacing_m <= 0.0) {
    out.clear();
    return;
  }

  out.resize(
    static_cast<std::size_t>(layer_count) * static_cast<std::size_t>(sample_count));
  const double denom = static_cast<double>(sample_count - 1);
  for (int layer = 0; layer < layer_count; ++layer) {
    const double layer_s0 = s_start + static_cast<double>(layer) * layer_spacing_m;
    for (int i = 0; i < sample_count; ++i) {
      const double s = layer_s0 + (static_cast<double>(i) / denom) * layer_spacing_m;
      out[static_cast<std::size_t>(layer) * static_cast<std::size_t>(sample_count) +
        static_cast<std::size_t>(i)] = sampleAtS(s);
    }
  }
}

double FrenetConverter::getRacingLineCurvatureDerivative(double s) const
{
  if (racing_line_.size() < 2 || total_length_ <= 1e-12 || waypoint_curvatures_.empty()) {
    return 0.0;
  }

  int n = static_cast<int>(racing_line_.size());
  s = wrapS(s);

  auto upper = std::upper_bound(distance_prefix_sum_.begin(), distance_prefix_sum_.end(), s);
  int i = static_cast<int>(std::distance(distance_prefix_sum_.begin(), upper)) - 1;
  i = std::clamp(i, 0, n - 1);
  int j = (i + 1) % n;

  double seg_len;
  if (j == 0) {
    seg_len = total_length_ - distance_prefix_sum_[i];
  } else {
    seg_len = distance_prefix_sum_[j] - distance_prefix_sum_[i];
  }

  if (seg_len < 1e-12) {return 0.0;}
  return (waypoint_curvatures_[j] - waypoint_curvatures_[i]) / seg_len;
}

} // namespace local_planning
