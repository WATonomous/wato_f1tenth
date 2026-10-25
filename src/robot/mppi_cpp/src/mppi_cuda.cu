// CUDA port of mppi_tracking.MPPI + InferEnv.reward_fn_xy.
#include <cuda_runtime.h>
#include <curand_kernel.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "mppi_cpp/dynamics.cuh"
#include "mppi_cpp/mppi_cuda.hpp"

namespace mppi {
namespace {

constexpr int kMaxSteps = 64;
constexpr int kRolloutBlock = 256;
constexpr int kWeightsBlock = 1024;

#define CUDA_CHECK(call)                                                            \
  do {                                                                              \
    cudaError_t err__ = (call);                                                     \
    if (err__ != cudaSuccess)                                                       \
      throw std::runtime_error(std::string("CUDA error: ") + cudaGetErrorString(err__)); \
  } while (0)

struct KernelArgs {
  int K, T, model, n_sub;
  uint64_t seed, offset;
  RuntimeParams rp;
  WallSdfView sdf;
  float x0[kStateDim];
  const float *a_opt, *ref, *opp, *mu;
  const float* fixed_actions;  // test hook: when set, use these instead of sampling
  float *da, *states, *R;
};

// jax.random.truncated_normal: standard normal restricted to [lo, hi].
__device__ inline float truncated_normal(curandStatePhilox4_32_10_t& st, float lo, float hi) {
  const float sqrt2 = 1.41421356237f;
  const float a = erff(lo / sqrt2), b = erff(hi / sqrt2);
  const float u = a + (b - a) * curand_uniform(&st);
  return fminf(fmaxf(sqrt2 * erfinvf(u), lo), hi);
}

__device__ inline float nan_to_num(float v) {
  if (isnan(v)) return 1e3f;
  if (isinf(v)) return v > 0.f ? 1e3f : -1e3f;
  return v;
}

__device__ inline float hinge_sq(float v, float thr) {
  const float d = fminf(fmaxf(0.f, v - thr), 1e3f);
  return d * d;
}

__device__ inline float sample_sdf(const WallSdfView& s, float x, float y) {
  const int col = static_cast<int>(floorf((x - s.ox) / s.res));
  const int row = static_cast<int>(floorf((y - s.oy) / s.res));
  if (row < 0 || row >= s.h || col < 0 || col >= s.w) return 0.f;
  return s.data[row * s.w + col];
}

__device__ float step_reward(const KernelArgs& a, const float* x, int t) {
  float invalid = 0.f, s[kStateDim];
  for (int i = 0; i < kStateDim; ++i) {
    if (!isfinite(x[i])) invalid = 1e3f;
    s[i] = nan_to_num(x[i]);
  }
  const RuntimeParams& p = a.rp;
  const float* r = a.ref + (t + 1) * kStateDim;
  const float xy = -(fabsf(r[0] - s[0]) + fabsf(r[1] - s[1]));
  const float vel = -fabsf(r[2] - s[3]);
  const float yaw = -fabsf(sinf(r[3]) - sinf(s[4])) - fabsf(cosf(r[3]) - cosf(s[4]));
  float reward = p.reward_weights[0] * xy + p.reward_weights[1] * vel +
                 p.reward_weights[2] * yaw - invalid;

  if (a.sdf.data) {
    const float d = sample_sdf(a.sdf, s[0], s[1]);
    reward -= p.wall_weight * powf(fmaxf(0.f, p.wall_margin - d), p.wall_power);
  }
  reward -= p.slip_weight * hinge_sq(fabsf(s[6]), p.beta_safe);
  reward -= p.latacc_weight * hinge_sq(fabsf(s[3] * s[5]), p.latacc_safe);
  reward -= p.steer_sat_weight * hinge_sq(fabsf(s[2]), p.steer_soft);

  const float dx = s[0] - a.opp[2 * t], dy = s[1] - a.opp[2 * t + 1];
  const float dist = sqrtf(dx * dx + dy * dy);
  reward -= p.opponent_weight * powf(p.opponent_discount, static_cast<float>(t)) *
            powf(fmaxf(0.f, p.opponent_radius - dist), p.opponent_power);
  return reward;
}

// One thread per sample: sample da, roll out, score, and accumulate returns.
__global__ void rollout_kernel(KernelArgs a) {
  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= a.K) return;
  curandStatePhilox4_32_10_t st;
  curand_init(a.seed, k, a.offset, &st);

  float x[kStateDim], r[kMaxSteps];
  for (int i = 0; i < kStateDim; ++i) x[i] = a.x0[i];

  for (int t = 0; t < a.T; ++t) {
    float act[kActionDim];
    for (int c = 0; c < kActionDim; ++c) {
      const float ao = a.a_opt[t * kActionDim + c];
      const float d = a.fixed_actions ? 0.f : truncated_normal(st, -a.rp.a_std[c] - ao, a.rp.a_std[c] - ao);
      a.da[(k * a.T + t) * kActionDim + c] = d;
      act[c] = a.fixed_actions ? a.fixed_actions[t * kActionDim + c] : fminf(fmaxf(ao + d, -1.f), 1.f);
    }
    step_state(x, act[0], act[1], a.rp.steer_scale, a.rp.accel_scale, a.mu[t], a.model, a.n_sub);
    float* out = a.states + (k * a.T + t) * kStateDim;
    for (int i = 0; i < kStateDim; ++i) out[i] = x[i];
    const float rt = step_reward(a, x, t);
    r[t] = isfinite(rt) ? rt : -1e6f;
  }
  float acc = 0.f;
  for (int t = a.T - 1; t >= 0; --t) {
    acc += r[t];
    a.R[k * a.T + t] = isfinite(acc) ? acc : -1e6f;
  }
}

struct MaxOp { __device__ float operator()(float a, float b) const { return fmaxf(a, b); } };
struct MinOp { __device__ float operator()(float a, float b) const { return fminf(a, b); } };
struct AddOp { __device__ float operator()(float a, float b) const { return a + b; } };

template <typename Op>
__device__ float block_reduce(float v, float* sh, Op op) {
  sh[threadIdx.x] = v;
  __syncthreads();
  for (int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (threadIdx.x < s) sh[threadIdx.x] = op(sh[threadIdx.x], sh[threadIdx.x + s]);
    __syncthreads();
  }
  const float out = sh[0];
  __syncthreads();
  return out;
}

// One block per timestep: softmax weights over samples, weighted mean of da.
__global__ void __launch_bounds__(kWeightsBlock) weights_kernel(const float* R, const float* da, int K, int T, float temperature,
                               float damping, float* da_opt) {
  __shared__ float sh[kWeightsBlock];
  const int t = blockIdx.x;

  float mx = -INFINITY, mn = INFINITY;
  for (int k = threadIdx.x; k < K; k += blockDim.x) {
    const float v = R[k * T + t];
    mx = fmaxf(mx, v);
    mn = fminf(mn, v);
  }
  mx = block_reduce(mx, sh, MaxOp{});
  mn = block_reduce(mn, sh, MinOp{});
  const float denom = fmaxf((mx - mn) + damping, 1e-6f);

  float wsum = 0.f, s0 = 0.f, s1 = 0.f, m0 = 0.f, m1 = 0.f;
  for (int k = threadIdx.x; k < K; k += blockDim.x) {
    const float w = expf((R[k * T + t] - mx) / denom / temperature);
    const float d0 = da[(k * T + t) * kActionDim], d1 = da[(k * T + t) * kActionDim + 1];
    wsum += w; s0 += w * d0; s1 += w * d1; m0 += d0; m1 += d1;
  }
  wsum = block_reduce(wsum, sh, AddOp{});
  s0 = block_reduce(s0, sh, AddOp{});
  s1 = block_reduce(s1, sh, AddOp{});
  m0 = block_reduce(m0, sh, AddOp{});
  m1 = block_reduce(m1, sh, AddOp{});
  if (threadIdx.x == 0) {
    if (wsum > 0.f) {
      da_opt[t * kActionDim] = s0 / wsum;
      da_opt[t * kActionDim + 1] = s1 / wsum;
    } else {
      da_opt[t * kActionDim] = m0 / K;
      da_opt[t * kActionDim + 1] = m1 / K;
    }
  }
}

}  // namespace

MppiCuda::MppiCuda(int n_samples, int n_steps, int model, float sim_dt, uint64_t seed)
    : K_(n_samples), T_(n_steps), model_(model), seed_(seed) {
  if (T_ < 1 || T_ > kMaxSteps) throw std::invalid_argument("n_steps must be in [1, 64]");
  n_sub_ = std::max(1, static_cast<int>(sim_dt / kSubDt));
  a_opt_.assign(T_ * kActionDim, 0.f);
  traj_opt_.assign(T_ * kStateDim, 0.f);
  CUDA_CHECK(cudaMalloc(&d_da_, sizeof(float) * K_ * T_ * kActionDim));
  CUDA_CHECK(cudaMalloc(&d_states_, sizeof(float) * K_ * T_ * kStateDim));
  CUDA_CHECK(cudaMalloc(&d_R_, sizeof(float) * K_ * T_));
  CUDA_CHECK(cudaMalloc(&d_da_opt_, sizeof(float) * T_ * kActionDim));
  CUDA_CHECK(cudaMalloc(&d_a_opt_, sizeof(float) * T_ * kActionDim));
  CUDA_CHECK(cudaMalloc(&d_ref_, sizeof(float) * (T_ + 1) * kStateDim));
  CUDA_CHECK(cudaMalloc(&d_opp_, sizeof(float) * T_ * 2));
  CUDA_CHECK(cudaMalloc(&d_mu_, sizeof(float) * T_));
}

MppiCuda::~MppiCuda() {
  for (float* p : {d_da_, d_states_, d_R_, d_da_opt_, d_a_opt_, d_ref_, d_opp_, d_mu_, d_sdf_})
    if (p) cudaFree(p);
}

void MppiCuda::set_wall_sdf(const std::vector<float>& sdf, int h, int w, float ox, float oy,
                            float res) {
  if (d_sdf_) { cudaFree(d_sdf_); d_sdf_ = nullptr; sdf_ = {}; }
  if (sdf.empty()) return;
  CUDA_CHECK(cudaMalloc(&d_sdf_, sizeof(float) * sdf.size()));
  CUDA_CHECK(cudaMemcpy(d_sdf_, sdf.data(), sizeof(float) * sdf.size(), cudaMemcpyHostToDevice));
  sdf_ = {d_sdf_, h, w, ox, oy, res};
}

void MppiCuda::reset_warm_start() { std::fill(a_opt_.begin(), a_opt_.end(), 0.f); }

void MppiCuda::update(const float* state, const std::vector<float>& reference,
                      const std::vector<float>& opponent, const std::vector<float>& mu,
                      const RuntimeParams& rp) {
  if (reference.size() != static_cast<size_t>((T_ + 1) * kStateDim) ||
      opponent.size() != static_cast<size_t>(T_ * 2) || mu.size() != static_cast<size_t>(T_))
    throw std::invalid_argument("MppiCuda::update: bad input sizes");
  CUDA_CHECK(cudaMemcpy(d_ref_, reference.data(), sizeof(float) * reference.size(), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_opp_, opponent.data(), sizeof(float) * opponent.size(), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_mu_, mu.data(), sizeof(float) * mu.size(), cudaMemcpyHostToDevice));

  // shift_prev_opt: drop the executed action, repeat the last one.
  std::copy(a_opt_.begin() + kActionDim, a_opt_.end(), a_opt_.begin());

  KernelArgs a{};
  a.K = K_; a.T = T_; a.model = model_; a.n_sub = n_sub_;
  a.seed = seed_; a.rp = rp; a.sdf = sdf_;
  for (int i = 0; i < kStateDim; ++i) a.x0[i] = state[i];
  a.a_opt = d_a_opt_; a.ref = d_ref_; a.opp = d_opp_; a.mu = d_mu_;
  a.da = d_da_; a.states = d_states_; a.R = d_R_;

  std::vector<float> da_opt(T_ * kActionDim);
  const int grid = (K_ + kRolloutBlock - 1) / kRolloutBlock;
  for (int it = 0; it < std::max(1, rp.n_iterations); ++it) {
    a.offset = offset_;
    offset_ += static_cast<uint64_t>(T_ * kActionDim);
    CUDA_CHECK(cudaMemcpy(d_a_opt_, a_opt_.data(), sizeof(float) * a_opt_.size(), cudaMemcpyHostToDevice));
    rollout_kernel<<<grid, kRolloutBlock>>>(a);
    weights_kernel<<<T_, kWeightsBlock>>>(d_R_, d_da_, K_, T_, rp.temperature, rp.damping, d_da_opt_);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaMemcpy(da_opt.data(), d_da_opt_, sizeof(float) * da_opt.size(), cudaMemcpyDeviceToHost));
    for (size_t i = 0; i < a_opt_.size(); ++i)
      a_opt_[i] = std::min(std::max(a_opt_[i] + da_opt[i], -1.f), 1.f);
  }

  if (rp.render) {
    traj_opt_ = rollout_host(a_opt_, state, mu, rp.steer_scale, rp.accel_scale);
  } else {
    CUDA_CHECK(cudaMemcpy(traj_opt_.data(), d_states_, sizeof(float) * traj_opt_.size(), cudaMemcpyDeviceToHost));
  }
}

void MppiCuda::eval(const float* state, const std::vector<float>& actions, const std::vector<float>& reference,
                    const std::vector<float>& opponent, const std::vector<float>& mu, const RuntimeParams& rp,
                    std::vector<float>& states_out, std::vector<float>& returns_out) {
  CUDA_CHECK(cudaMemcpy(d_ref_, reference.data(), sizeof(float) * reference.size(), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_opp_, opponent.data(), sizeof(float) * opponent.size(), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_mu_, mu.data(), sizeof(float) * mu.size(), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_a_opt_, actions.data(), sizeof(float) * actions.size(), cudaMemcpyHostToDevice));
  KernelArgs a{};
  a.K = 1; a.T = T_; a.model = model_; a.n_sub = n_sub_;
  a.seed = seed_; a.rp = rp; a.sdf = sdf_;
  for (int i = 0; i < kStateDim; ++i) a.x0[i] = state[i];
  a.a_opt = d_a_opt_; a.ref = d_ref_; a.opp = d_opp_; a.mu = d_mu_; a.fixed_actions = d_a_opt_;
  a.da = d_da_; a.states = d_states_; a.R = d_R_;
  rollout_kernel<<<1, 1>>>(a);
  CUDA_CHECK(cudaGetLastError());
  states_out.resize(T_ * kStateDim);
  returns_out.resize(T_);
  CUDA_CHECK(cudaMemcpy(states_out.data(), d_states_, sizeof(float) * states_out.size(), cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(returns_out.data(), d_R_, sizeof(float) * returns_out.size(), cudaMemcpyDeviceToHost));
}

std::vector<float> MppiCuda::returns() const {
  std::vector<float> out(static_cast<size_t>(K_) * T_);
  CUDA_CHECK(cudaMemcpy(out.data(), d_R_, sizeof(float) * out.size(), cudaMemcpyDeviceToHost));
  return out;
}

std::vector<float> MppiCuda::perturbations() const {
  std::vector<float> out(static_cast<size_t>(K_) * T_ * kActionDim);
  CUDA_CHECK(cudaMemcpy(out.data(), d_da_, sizeof(float) * out.size(), cudaMemcpyDeviceToHost));
  return out;
}

std::vector<float> MppiCuda::sampled_states() const {
  std::vector<float> out(static_cast<size_t>(K_) * T_ * kStateDim);
  CUDA_CHECK(cudaMemcpy(out.data(), d_states_, sizeof(float) * out.size(), cudaMemcpyDeviceToHost));
  return out;
}

std::vector<float> MppiCuda::rollout_host(const std::vector<float>& actions, const float* state,
                                          const std::vector<float>& mu, float steer_scale,
                                          float accel_scale) const {
  std::vector<float> out(T_ * kStateDim);
  float x[kStateDim];
  for (int i = 0; i < kStateDim; ++i) x[i] = state[i];
  for (int t = 0; t < T_; ++t) {
    step_state(x, actions[t * kActionDim], actions[t * kActionDim + 1], steer_scale, accel_scale,
               mu[t], model_, n_sub_);
    for (int i = 0; i < kStateDim; ++i) out[t * kStateDim + i] = x[i];
  }
  return out;
}

}  // namespace mppi
