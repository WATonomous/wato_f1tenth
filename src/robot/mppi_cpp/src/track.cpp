#include "mppi_cpp/track.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace mppi {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kFrictionMin = 1e-3;

double pymod(double a, double b) { return a - std::floor(a / b) * b; }
double wrap_pi(double a) { return pymod(a + kPi, 2.0 * kPi) - kPi; }
int pymod(int a, int b) { return ((a % b) + b) % b; }
}  // namespace

Track Track::load_csv(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open raceline csv: " + path);
  std::vector<std::vector<double>> rows;
  std::string line;
  for (int i = 0; i < 3 && std::getline(in, line); ++i) {}  // wpt_rowskip = 3
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::vector<double> row;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ';')) row.push_back(std::stod(tok));
    if (!rows.empty() && row.size() != rows.front().size())
      throw std::runtime_error("ragged raceline csv: " + path);
    rows.push_back(std::move(row));
  }
  if (rows.size() < 2 || rows.front().size() < 7)
    throw std::runtime_error("expected waypoints as [s, x, y, psi, k, vx, ax]: " + path);

  Track t;
  const int N = static_cast<int>(rows.size()), C = static_cast<int>(rows.front().size());
  t.wp_.resize(N, C);
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < C; ++j) t.wp_(i, j) = rows[i][j];
  for (int i = 0; i + 1 < N; ++i) {
    t.diff_x_.push_back(t.wp_(i + 1, 1) - t.wp_(i, 1));
    t.diff_y_.push_back(t.wp_(i + 1, 2) - t.wp_(i, 2));
    t.seg_len_.push_back(std::hypot(t.diff_x_.back(), t.diff_y_.back()));
  }
  if (C >= 10) {
    t.frictions_.resize(N);
    bool finite = true;
    for (int i = 0; i < N; ++i) {
      t.frictions_[i] = t.wp_(i, 9);
      finite = finite && std::isfinite(t.frictions_[i]);
    }
    t.has_friction_ = finite;
  }
  return t;
}

NearestPoint Track::nearest(double x, double y) const {
  const int M = static_cast<int>(seg_len_.size());
  double best = std::numeric_limits<double>::infinity(), best_t = 0.0;
  int best_i = 0;
  for (int i = 0; i < M; ++i) {
    const double l2 = diff_x_[i] * diff_x_[i] + diff_y_[i] * diff_y_[i];
    const double dot = (x - wp_(i, 1)) * diff_x_[i] + (y - wp_(i, 2)) * diff_y_[i];
    const double t = std::clamp(dot / (l2 + 1e-8), 0.0, 1.0);
    const double px = wp_(i, 1) + t * diff_x_[i], py = wp_(i, 2) + t * diff_y_[i];
    const double d = std::hypot(x - px, y - py);
    if (d < best) { best = d; best_t = t; best_i = i; }
  }
  return {std::hypot(diff_x_[best_i] * best_t, diff_y_[best_i] * best_t), best, best_i};
}

// get_reference_trajectory (numpy version in infer_env.py).
RowMat Track::interpolate(const std::vector<double>& speeds, double dist_from_segment_start,
                          int idx, int n_steps, double dt) const {
  const int M = static_cast<int>(seg_len_.size());  // N - 1
  std::vector<double> s_rel(n_steps + 1);
  s_rel[0] = dist_from_segment_start;
  for (int i = 1; i <= n_steps; ++i) s_rel[i] = s_rel[i - 1] + speeds[i - 1] * dt;

  std::vector<double> wdr(M);  // cumsum(roll(seg_len, -idx))
  double acc = 0.0;
  for (int j = 0; j < M; ++j) { acc += seg_len_[pymod(idx + j, M)]; wdr[j] = acc; }

  RowMat ref(n_steps + 1, kRefDim);
  const double s_max = wp_(wp_.rows() - 1, 0);
  for (int i = 0; i <= n_steps; ++i) {
    int ir = static_cast<int>(std::upper_bound(wdr.begin(), wdr.end(), s_rel[i]) - wdr.begin());
    ir = std::min(ir, M - 1);
    const int ia = pymod(idx + ir, M), nx = pymod(ia + 1, M);
    const double segment_part = s_rel[i] - (wdr[ir] - seg_len_[ia]);
    const double t = segment_part / seg_len_[ia];
    ref(i, 0) = wp_(ia, 1) + t * (wp_(nx, 1) - wp_(ia, 1));
    ref(i, 1) = wp_(ia, 2) + t * (wp_(nx, 2) - wp_(ia, 2));
    ref(i, 2) = wp_(ia, 5) + t * (wp_(nx, 5) - wp_(ia, 5));
    ref(i, 3) = wrap_pi(wp_(ia, 3) + t * (wp_(nx, 3) - wp_(ia, 3)));
    double s = wp_(ia, 0) + t * (wp_(nx, 0) - wp_(ia, 0));
    if (s > s_max) s -= s_max;
    ref(i, 4) = s;
    ref(i, 5) = 0.0;
    ref(i, 6) = 0.0;
  }
  return ref;
}

std::vector<double> Track::profiled_speeds(const std::vector<double>& speeds_in,
                                           const Config& cfg) const {
  const double lo = cfg.speed_profile_min_speed;
  const double hi = std::max(lo, cfg.speed_profile_max_speed);
  std::vector<double> speeds(speeds_in.size());
  for (size_t i = 0; i < speeds.size(); ++i)
    speeds[i] = std::clamp(speeds_in[i] * cfg.speed_profile_scale, lo, hi);
  const int L = std::max(0, cfg.speed_profile_lookahead_steps);
  if (L > 0 && speeds.size() > 1) {
    std::vector<double> out(speeds.size());
    const int n = static_cast<int>(speeds.size());
    for (int i = 0; i < n; ++i) {
      double m = speeds[i];
      for (int j = i + 1; j <= i + L; ++j) m = std::min(m, speeds[std::min(j, n - 1)]);
      out[i] = m;
    }
    speeds = out;
  }
  return speeds;
}

RowMat Track::reference(const double* state, double target_speed, int n_steps, const Config& cfg,
                        int* index_out) const {
  const NearestPoint np = nearest(state[0], state[1]);
  double speed = target_speed;
  if (cfg.use_waypoint_speed_profile) speed = profiled_speeds({wp_(np.index, 5)}, cfg)[0];

  std::vector<double> speeds(n_steps, speed);
  RowMat ref = interpolate(speeds, np.dist_from_segment_start, np.index, n_steps, cfg.sim_time_step);
  if (cfg.use_waypoint_speed_profile) {
    auto col2 = [&]() {
      std::vector<double> v(n_steps + 1);
      for (int i = 0; i <= n_steps; ++i) v[i] = ref(i, 2);
      return v;
    };
    for (int it = 0; it < std::max(1, cfg.speed_profile_iterations); ++it) {
      const std::vector<double> targets = profiled_speeds(col2(), cfg);
      speeds.assign(targets.begin() + 1, targets.begin() + 1 + n_steps);
      ref = interpolate(speeds, np.dist_from_segment_start, np.index, n_steps, cfg.sim_time_step);
    }
    const std::vector<double> final_speeds = profiled_speeds(col2(), cfg);
    for (int i = 0; i <= n_steps; ++i) ref(i, 2) = final_speeds[i];
  }

  const double orientation = state[4];
  for (int i = 0; i <= n_steps; ++i) {
    if (ref(i, 3) - orientation > 5.0) ref(i, 3) = std::abs(ref(i, 3) - 2.0 * kPi);
    if (ref(i, 3) - orientation < -5.0) ref(i, 3) = std::abs(ref(i, 3) + 2.0 * kPi);
  }
  if (index_out) *index_out = np.index;
  return ref;
}

std::vector<float> Track::reference_frictions(const double* state, int n_steps,
                                              const Config& cfg) const {
  const double hi = cfg.friction_max;
  if (!has_friction_)
    return std::vector<float>(n_steps, static_cast<float>(std::clamp(cfg.friction, kFrictionMin, hi)));
  const int N = static_cast<int>(frictions_.size());
  const int ind = nearest(state[0], state[1]).index;
  std::vector<float> mu(n_steps);
  for (int i = 0; i < n_steps; ++i)
    mu[i] = static_cast<float>(std::clamp(frictions_[(ind + i) % N], kFrictionMin, hi));
  return mu;
}

}  // namespace mppi
