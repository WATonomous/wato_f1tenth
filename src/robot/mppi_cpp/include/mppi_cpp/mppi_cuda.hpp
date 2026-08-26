#pragma once
#include <cstdint>
#include <vector>

namespace mppi {

constexpr int kStateDim = 7;   // x, y, delta, vx, yaw, yaw_rate, beta
constexpr int kActionDim = 2;  // steer rate, accel
enum Model : int { DYNAMIC_ST = 0, KINEMATIC_ST = 1 };

// Per-call tunables (MPPI.runtime_params in Python).
struct RuntimeParams {
  float a_std[2];
  float temperature, damping;
  float reward_weights[3];  // xy, velocity, yaw
  float wall_weight, wall_margin, wall_power;
  float slip_weight, beta_safe;
  float latacc_weight, latacc_safe;
  float steer_sat_weight, steer_soft;
  float opponent_weight, opponent_radius, opponent_power, opponent_discount;
  float steer_scale, accel_scale;
  int n_iterations;
  bool render;
};

struct WallSdfView {
  const float* data = nullptr;  // row-major [h][w], row 0 = origin y
  int h = 0, w = 0;
  float ox = 0.f, oy = 0.f, res = 1.f;
};

class MppiCuda {
 public:
  MppiCuda(int n_samples, int n_steps, int model, float sim_dt, uint64_t seed);
  ~MppiCuda();
  MppiCuda(const MppiCuda&) = delete;
  MppiCuda& operator=(const MppiCuda&) = delete;

  void set_wall_sdf(const std::vector<float>& sdf, int h, int w, float ox, float oy, float res);

  // state: 7, reference: (n_steps+1)*7 row-major, opponent: n_steps*2, mu: n_steps.
  void update(const float* state, const std::vector<float>& reference,
              const std::vector<float>& opponent, const std::vector<float>& mu,
              const RuntimeParams& rp);

  void reset_warm_start();

  // Test hooks: roll out fixed actions on the GPU (no sampling) and read back
  // the last update's per-sample returns / perturbations.
  void eval(const float* state, const std::vector<float>& actions, const std::vector<float>& reference,
            const std::vector<float>& opponent, const std::vector<float>& mu, const RuntimeParams& rp,
            std::vector<float>& states_out, std::vector<float>& returns_out);
  std::vector<float> returns() const;        // n_samples*n_steps
  std::vector<float> perturbations() const;  // n_samples*n_steps*2
  const std::vector<float>& a_opt() const { return a_opt_; }   // n_steps*2
  const std::vector<float>& traj_opt() const { return traj_opt_; }  // n_steps*7
  std::vector<float> sampled_states() const;  // n_samples*n_steps*7 (D2H copy)

  // Deterministic single rollout on the host (same dynamics as the kernels).
  std::vector<float> rollout_host(const std::vector<float>& actions, const float* state,
                                  const std::vector<float>& mu, float steer_scale,
                                  float accel_scale) const;

 private:
  int K_, T_, model_, n_sub_;
  uint64_t seed_, offset_ = 0;
  std::vector<float> a_opt_, traj_opt_;
  float *d_da_ = nullptr, *d_states_ = nullptr, *d_R_ = nullptr, *d_da_opt_ = nullptr;
  float *d_a_opt_ = nullptr, *d_ref_ = nullptr, *d_opp_ = nullptr, *d_mu_ = nullptr;
  float* d_sdf_ = nullptr;
  WallSdfView sdf_;
};

}  // namespace mppi
