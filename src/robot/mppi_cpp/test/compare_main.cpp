// A/B harness: prints C++ results for fixed inputs; test/compare_with_jax.py
// computes the same with the original JAX code and diffs them.
// Usage: compare_main <raceline.csv> <map.yaml>
#include <cmath>
#include <cstdio>
#include <vector>

#include "mppi_cpp/config.hpp"
#include "mppi_cpp/mppi_cuda.hpp"
#include "mppi_cpp/track.hpp"
#include "mppi_cpp/wall_sdf.hpp"

using namespace mppi;

static void print(const char* name, const std::vector<float>& v) {
  std::printf("%s", name);
  for (float x : v) std::printf(" %.9g", x);
  std::printf("\n");
}
static void print(const char* name, const RowMat& m) {
  std::printf("%s", name);
  for (Eigen::Index i = 0; i < m.rows(); ++i)
    for (Eigen::Index j = 0; j < m.cols(); ++j) std::printf(" %.12g", m(i, j));
  std::printf("\n");
}

int main(int argc, char** argv) {
  if (argc < 3) return 1;
  Config cfg;
  cfg.n_steps = 12; cfg.sim_time_step = 0.1; cfg.n_samples = 8192;
  cfg.use_waypoint_speed_profile = true; cfg.speed_profile_scale = 0.4;
  cfg.speed_profile_min_speed = 1.0; cfg.speed_profile_max_speed = 9.0;
  cfg.speed_profile_lookahead_steps = 3; cfg.speed_profile_iterations = 3;
  cfg.friction = 0.5; cfg.friction_max = 1.5; cfg.ref_vel = 2.0;
  const int T = cfg.n_steps;

  Track track = Track::load_csv(argv[1]);
  WallSdf sdf = WallSdf::load(argv[2]);
  MppiCuda mppi(cfg.n_samples, T, DYNAMIC_ST, cfg.sim_time_step, 1234);
  mppi.set_wall_sdf(sdf.data, sdf.h, sdf.w, sdf.ox, sdf.oy, sdf.res);

  const double states[3][7] = {{11.2, -0.55, 0.05, 2.0, -0.05, 0.1, 0.02},
                               {5.0, 3.0, -0.1, 4.0, 2.0, -0.5, -0.05},
                               {0.0, 0.0, 0.0, 1.5, 1.0, 0.0, 0.0}};
  std::vector<RowMat> refs;
  for (int i = 0; i < 3; ++i) {
    refs.push_back(track.reference(states[i], std::max(cfg.ref_vel, states[i][3]), T, cfg));
    char name[16]; std::snprintf(name, sizeof name, "ref%d", i);
    print(name, refs.back());
  }

  std::vector<float> probes;
  for (int i = 0; i < 200; ++i) {
    const float x = sdf.ox - 1.f + (sdf.w * sdf.res + 2.f) * (i % 20) / 19.f;
    const float y = sdf.oy - 1.f + (sdf.h * sdf.res + 2.f) * (i / 20) / 9.f;
    probes.push_back(sdf.sample(x, y));
  }
  print("sdf", probes);

  std::vector<float> actions(T * 2);
  for (int t = 0; t < T; ++t) {
    actions[2 * t] = 0.6f * std::sin(0.7f * t + 0.3f);
    actions[2 * t + 1] = 0.5f * std::cos(0.5f * t);
  }
  RuntimeParams rp{};
  rp.a_std[0] = 0.8f; rp.a_std[1] = 10.f; rp.temperature = 0.015f; rp.damping = 0.001f;
  rp.reward_weights[0] = 0.25f; rp.reward_weights[1] = 0.5f; rp.reward_weights[2] = 0.01f;
  rp.wall_weight = 10.f; rp.wall_margin = 0.3f; rp.wall_power = 0.6f;
  rp.slip_weight = 2.f; rp.beta_safe = 0.4f; rp.latacc_weight = 3.f; rp.latacc_safe = 80.f;
  rp.steer_sat_weight = 0.5f; rp.steer_soft = 0.425f;
  rp.opponent_weight = 20.f; rp.opponent_radius = 0.4f; rp.opponent_power = 0.4f; rp.opponent_discount = 0.9f;
  rp.steer_scale = 2.7f; rp.accel_scale = 2.7f; rp.n_iterations = 1; rp.render = true;

  for (int i = 0; i < 2; ++i) {
    float s[7]; for (int j = 0; j < 7; ++j) s[j] = states[i][j];
    const std::vector<float> mu = track.reference_frictions(states[i], T, cfg);
    std::vector<float> ref((T + 1) * 7), opp(T * 2);
    for (int r = 0; r <= T; ++r) for (int c = 0; c < 7; ++c) ref[r * 7 + c] = refs[i](r, c);
    for (int t = 0; t < T; ++t) { opp[2 * t] = refs[i](t + 1, 0) + 0.2f; opp[2 * t + 1] = refs[i](t + 1, 1) + 0.1f; }
    std::vector<float> st, R;
    mppi.eval(s, actions, ref, opp, mu, rp, st, R);
    char name[16];
    std::snprintf(name, sizeof name, "roll%d", i); print(name, st);
    std::snprintf(name, sizeof name, "hroll%d", i); print(name, mppi.rollout_host(actions, s, mu, rp.steer_scale, rp.accel_scale));
    std::snprintf(name, sizeof name, "R%d", i); print(name, R);

    // Full update: check a_opt update == host-side softmax-weighted average of da.
    mppi.reset_warm_start();
    mppi.update(s, ref, opp, mu, rp);
    const std::vector<float> a_new = mppi.a_opt(), Rall = mppi.returns(), da = mppi.perturbations();
    double max_err = 0.0;
    for (int t = 0; t < T; ++t) {
      double mx = -1e300, mn = 1e300;
      for (int k = 0; k < cfg.n_samples; ++k) { mx = std::max(mx, (double)Rall[k * T + t]); mn = std::min(mn, (double)Rall[k * T + t]); }
      const double denom = std::max(mx - mn + rp.damping, 1e-6);
      double ws = 0, s0 = 0, s1 = 0;
      for (int k = 0; k < cfg.n_samples; ++k) {
        const double w = std::exp((Rall[k * T + t] - mx) / denom / rp.temperature);
        ws += w; s0 += w * da[(k * T + t) * 2]; s1 += w * da[(k * T + t) * 2 + 1];
      }
      max_err = std::max({max_err, std::abs(std::clamp(s0 / ws, -1.0, 1.0) - a_new[2 * t]),
                          std::abs(std::clamp(s1 / ws, -1.0, 1.0) - a_new[2 * t + 1])});
    }
    double da_min = 1e9, da_max = -1e9;
    for (size_t k = 0; k < da.size(); k += 2) { da_min = std::min(da_min, (double)da[k]); da_max = std::max(da_max, (double)da[k]); }
    std::printf("weights_err%d %.3g %.4f %.4f\n", i, max_err, da_min, da_max);
    std::snprintf(name, sizeof name, "aopt%d", i); print(name, a_new);
  }
  return 0;
}
