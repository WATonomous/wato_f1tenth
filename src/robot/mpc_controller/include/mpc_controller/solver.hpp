#pragma once
#include "mpc_controller/model.hpp"
#include <chrono>

struct mpc_proto_solver_capsule;
namespace mpc {
using Clock = std::chrono::steady_clock;
struct Face {double x = 0, y = 0, bound = 0;};
struct Problem {
  Pose frame;
  Command previous;
  double first_interval = .02;
  uint64_t scene_version = 0, track_version = 0;
  std::array<Pose, horizon+1> targets{};
  std::array<double, horizon> speeds{};
  std::array<std::array<Face, 4>, horizon+1> corridor{};
  std::array<State, horizon+1> seed_states{};
  std::array<Command, horizon> seed_commands{};
};
struct Solution {
  Pose frame;
  uint64_t scene_version = 0, track_version = 0;
  int status = -1;
  double solve_ms = 0, inequality_residual = INFINITY, dynamics_residual = INFINITY;
  std::array<State, horizon+1> states{};
  std::array<Command, horizon> commands{};
};
class Solver {
public:
  Solver();
  ~Solver();
  Solver(const Solver &) = delete;
  Solver & operator=(const Solver &) = delete;
  Solution solve(const Problem & problem);
  void reset();
private:
  mpc_proto_solver_capsule * capsule_;
};
}  // namespace mpc
