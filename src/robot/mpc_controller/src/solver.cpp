#include "mpc_controller/solver.hpp"
#include "acados_solver_mpc_proto.h"

namespace mpc {
Solver::Solver() : capsule_(mpc_proto_acados_create_capsule())
{
  if (!capsule_ || mpc_proto_acados_create(capsule_) != 0) {
    throw std::runtime_error("Could not create acados solver");
  }
}
Solver::~Solver()
{
  mpc_proto_acados_free(capsule_);
  mpc_proto_acados_free_capsule(capsule_);
}
void Solver::reset()
{mpc_proto_acados_reset(capsule_, 1, 1, 0, 0);}

Solution Solver::solve(const Problem & problem)
{
  Solution result;
  result.frame = problem.frame;
  result.scene_version = problem.scene_version;
  result.track_version = problem.track_version;
  if (!finite(problem.frame) || !validCommand(problem.previous) ||
    !std::isfinite(problem.first_interval) || problem.first_interval <= 0) {return result;}
  for (int k = 0; k <= horizon; ++k) {
    if (!finite(problem.targets[k])) {return result;}
    for (double value : problem.seed_states[k]) {
      if (!std::isfinite(value)) {return result;}
    }
    for (auto face : problem.corridor[k]) {
      if (!std::isfinite(face.x) || !std::isfinite(face.y) ||
        !std::isfinite(face.bound) || std::abs(std::hypot(face.x, face.y)-1) > 1e-6) {
        return result;
      }
    }
    if (k < horizon && (!validCommand(problem.seed_commands[k]) ||
      !std::isfinite(problem.speeds[k]) || problem.speeds[k] < 0 ||
      problem.speeds[k] > speed_cap)) {return result;}
  }
  const auto start = Clock::now();
  auto * config = mpc_proto_acados_get_nlp_config(capsule_);
  auto * dims = mpc_proto_acados_get_nlp_dims(capsule_);
  auto * in = mpc_proto_acados_get_nlp_in(capsule_);
  auto * out = mpc_proto_acados_get_nlp_out(capsule_);
  auto * solver = mpc_proto_acados_get_nlp_solver(capsule_);
  double initial[] = {0, 0, 0, problem.previous.speed, problem.previous.steering};
  ocp_nlp_constraints_model_set(config, dims, in, out, 0, "lbx", initial);
  ocp_nlp_constraints_model_set(config, dims, in, out, 0, "ubx", initial);
  for (int k = 0; k <= horizon; ++k) {
    auto target = problem.targets[k];
    double params[17] = {target.x, target.y, target.yaw,
      k < horizon ? problem.speeds[k] : 0, k == 0 ? problem.first_interval : dt};
    for (int h = 0; h < 4; ++h) {
      auto face = problem.corridor[k][h];
      const auto & seed=problem.seed_states[k];
      const auto center=coveringCenter({seed[0],seed[1],seed[2]});
      const double slack=face.bound-face.x*center.x-face.y*center.y;
      params[5+3*h] = face.x;
      params[6+3*h] = face.y;
      // Keep a feasible seed feasible; a fixed contraction can exclude the stopped car.
      params[7+3*h] = face.bound-(k==0 ? 0 : std::min(.002,std::max(0.,slack/2)));
    }
    mpc_proto_acados_update_params(capsule_, k, params, 17);
    State guess = problem.seed_states[k];
    ocp_nlp_out_set(config, dims, out, in, k, "x", guess.data());
    if (k < horizon) {
      auto u = problem.seed_commands[k];
      double guess_u[] = {u.speed, u.steering};
      double low[] = {0, steering_min}, high[] = {k == horizon-1 ? 0 : speed_cap, steering_max};
      ocp_nlp_constraints_model_set(config, dims, in, out, k, "lbu", low);
      ocp_nlp_constraints_model_set(config, dims, in, out, k, "ubu", high);
      ocp_nlp_out_set(config, dims, out, in, k, "u", guess_u);
    }
  }
  for (int iteration=0;iteration<3;++iteration) {
    result.status = mpc_proto_acados_solve(capsule_);
    ocp_nlp_eval_residuals(solver, in, out);
    ocp_nlp_get(solver, "res_ineq", &result.inequality_residual);
    ocp_nlp_get(solver, "res_eq", &result.dynamics_residual);
    if (result.status!=0 || (result.inequality_residual<=1e-6 && result.dynamics_residual<=1e-6)) {break;}
  }
  for (int k = 0; k <= horizon; ++k) {
    ocp_nlp_out_get(config, dims, out, k, "x", result.states[k].data());
    if (k < horizon) {
      double u[2];
      ocp_nlp_out_get(config, dims, out, k, "u", u);
      // Canonicalize only solver roundoff before independent rollout/validation.
      if (u[0] >= -1e-8 && u[0] <= speed_cap+1e-8) {u[0] = std::clamp(u[0],0.,speed_cap);}
      if (u[1] >= steering_min-1e-8 && u[1] <= steering_max+1e-8) {
        u[1] = std::clamp(u[1],steering_min,steering_max);
      }
      // Validate the actual float32 ROS payload, including inward boundary rounding.
      if (validCommand({u[0],u[1]})) {
        float speed=static_cast<float>(u[0]),steer=static_cast<float>(u[1]);
        if (speed>speed_cap) {speed=std::nextafter(speed,0.f);}
        if (steer<steering_min) {steer=std::nextafter(steer,INFINITY);}
        if (steer>steering_max) {steer=std::nextafter(steer,-INFINITY);}
        u[0]=speed;u[1]=steer;
      }
      result.commands[k] = {u[0], u[1]};
    }
  }
  result.solve_ms = std::chrono::duration<double, std::milli>(Clock::now()-start).count();
  return result;
}
}  // namespace mpc
