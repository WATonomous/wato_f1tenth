#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

#include "mppi_cpp/config.hpp"

namespace mppi {

using RowMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
constexpr int kRefDim = 7;  // x, y, v, yaw, s, 0, 0

struct NearestPoint {
  double dist_from_segment_start;
  double dist;
  int index;
};

class Track {
 public:
  // Levine-format raceline CSV: 3 skipped rows, ';' delimited,
  // s; x; y; psi; kappa; vx; ax; [w_right; w_left; [friction]]
  static Track load_csv(const std::string& path);

  NearestPoint nearest(double x, double y) const;

  // InferEnv.get_refernece_traj: (n_steps+1) x 7 reference along the raceline.
  RowMat reference(const double* state, double target_speed, int n_steps, const Config& cfg,
                   int* index_out = nullptr) const;

  // InferEnv.get_reference_frictions: n_steps per-step friction, clipped to [1e-3, friction_max].
  std::vector<float> reference_frictions(const double* state, int n_steps, const Config& cfg) const;

  const RowMat& waypoints() const { return wp_; }
  bool has_friction() const { return has_friction_; }

 private:
  RowMat wp_;                      // N x C
  std::vector<double> diff_x_, diff_y_, seg_len_;  // N-1
  std::vector<double> frictions_;  // N, raw
  bool has_friction_ = false;

  RowMat interpolate(const std::vector<double>& speeds, double dist_from_segment_start,
                     int idx, int n_steps, double dt) const;
  std::vector<double> profiled_speeds(const std::vector<double>& speeds, const Config& cfg) const;
};

}  // namespace mppi
