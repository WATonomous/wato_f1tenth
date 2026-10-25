#pragma once
// F1TENTH single-track models, port of dynamics_models_jax.py (params_f1tenth).
// Compiled for both host and device so visualization rollouts match the GPU.
#include <cmath>

#include "mppi_cpp/mppi_cuda.hpp"

#ifdef __CUDACC__
#define MPPI_HD __host__ __device__
#else
#define MPPI_HD
#endif

namespace mppi {

constexpr float kSubDt = 0.1f;  // fixed RK4 substep (Ddt in InferEnv)

namespace veh {
constexpr float C_Sf = 4.718f, C_Sr = 5.4562f, lf = 0.15875f, lr = 0.17145f, h = 0.074f;
constexpr float m = 3.74f, I = 0.04712f, s_min = -0.4189f, s_max = 0.4189f;
constexpr float sv_min = -3.2f, sv_max = 3.2f, v_switch = 7.319f, a_max = 9.51f;
constexpr float v_min = -5.0f, v_max = 20.0f, g = 9.81f;
}  // namespace veh

MPPI_HD inline float accl_constraints(float vel, float accl) {
  using namespace veh;
  const float pos_limit = vel > v_switch ? a_max * v_switch / vel : a_max;
  if (vel <= v_min && accl <= 0.f) accl = 0.f;
  if (vel >= v_max && accl >= 0.f) accl = 0.f;
  if (accl <= -a_max) accl = -a_max;
  if (accl >= pos_limit) accl = pos_limit;
  return accl;
}

MPPI_HD inline float steering_constraint(float angle, float rate) {
  using namespace veh;
  if (angle <= s_min && rate <= 0.f) rate = 0.f;
  if (angle >= s_max && rate >= 0.f) rate = 0.f;
  if (rate <= sv_min) rate = sv_min;
  if (rate >= sv_max) rate = sv_max;
  return rate;
}

MPPI_HD inline void dynamics_ks(const float* x, const float* u_in, float* f) {
  const float u0 = steering_constraint(x[2], u_in[0]);
  const float u1 = accl_constraints(x[3], u_in[1]);
  const float lwb = veh::lf + veh::lr;
  f[0] = x[3] * cosf(x[4]);
  f[1] = x[3] * sinf(x[4]);
  f[2] = u0;
  f[3] = u1;
  f[4] = x[3] / lwb * tanf(x[2]);
}

MPPI_HD inline void dynamics_st(const float* x, const float* u_in, float mu, float* f) {
  using namespace veh;
  const float u0 = steering_constraint(x[2], u_in[0]);
  const float u1 = accl_constraints(x[3], u_in[1]);
  const float u[2] = {u0, u1};
  const float lwb = lf + lr;
  if (fabsf(x[3]) < 1.f) {
    dynamics_ks(x, u, f);
    const float c = cosf(x[2]);
    f[5] = u1 / lwb * tanf(x[2]) + x[3] / (lwb * c * c) * u0;
    f[6] = 0.f;
    return;
  }
  const float vx = x[3], wz = x[5], beta = x[6], delta = x[2];
  const float glr = g * lr - u1 * h, glf = g * lf + u1 * h;
  f[0] = vx * cosf(beta + x[4]);
  f[1] = vx * sinf(beta + x[4]);
  f[2] = u0;
  f[3] = u1;
  f[4] = wz;
  f[5] = -mu * m / (vx * I * lwb) * (lf * lf * C_Sf * glr + lr * lr * C_Sr * glf) * wz
       + mu * m / (I * lwb) * (lr * C_Sr * glf - lf * C_Sf * glr) * beta
       + mu * m / (I * lwb) * lf * C_Sf * glr * delta;
  f[6] = (mu / (vx * vx * lwb) * (C_Sr * glf * lr - C_Sf * glr * lf) - 1.f) * wz
       - mu / (vx * lwb) * (C_Sr * glf + C_Sf * glr) * beta
       + mu / (vx * lwb) * (C_Sf * glr) * delta;
}

MPPI_HD inline void eval_dynamics(const float* x, const float* u, float mu, int model, float* f) {
  if (model == KINEMATIC_ST) dynamics_ks(x, u, f); else dynamics_st(x, u, mu, f);
}

// One RK4 substep of length kSubDt. Kinematic model only advances x[0:5].
MPPI_HD inline void rk4_step(float* x, const float* u, float mu, int model) {
  const int n = model == KINEMATIC_ST ? 5 : kStateDim;
  float k1[kStateDim], k2[kStateDim], k3[kStateDim], k4[kStateDim], xt[kStateDim];
  auto f = [&](const float* xs, float* out) { eval_dynamics(xs, u, mu, model, out); };
  f(x, k1);
  for (int i = 0; i < n; ++i) xt[i] = x[i] + k1[i] * 0.5f * kSubDt;
  for (int i = n; i < kStateDim; ++i) xt[i] = x[i];
  f(xt, k2);
  for (int i = 0; i < n; ++i) xt[i] = x[i] + k2[i] * 0.5f * kSubDt;
  f(xt, k3);
  for (int i = 0; i < n; ++i) xt[i] = x[i] + k3[i] * kSubDt;
  f(xt, k4);
  for (int i = 0; i < n; ++i) x[i] += (k1[i] + 2.f * k2[i] + 2.f * k3[i] + k4[i]) / 6.f * kSubDt;
}

// InferEnv.step: apply physical scaling to the normalized action, then n_sub RK4 substeps.
MPPI_HD inline void step_state(float* x, float a_steer, float a_accel, float steer_scale,
                               float accel_scale, float mu, int model, int n_sub) {
  const float u[2] = {a_steer * steer_scale, a_accel * accel_scale};
  for (int i = 0; i < n_sub; ++i) rk4_step(x, u, mu, model);
}

}  // namespace mppi
