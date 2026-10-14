#pragma once
#include "mpc_controller/scene.hpp"
#include <optional>
#include <string>

namespace mpc {
struct Validation {bool accepted=false; std::string reason; double minimum_clearance=INFINITY;};
Validation validate(const Problem &, const Solution &, const Scene &, Clock::time_point);
struct Cycle {
  Problem problem;
  Solution solution;
  Validation validation;
  double prepare_ms=0, validation_ms=0, total_ms=0;
};
class Controller {
public:
  Cycle cycle(Pose ego, Command previous, double elapsed, const Reference & reference,
    const Scene & scene, Clock::time_point deadline);
  void reset();
  const std::vector<Pose> & route() const {return route_;}
private:
  Problem prepare(Pose ego, Command previous, double elapsed, const Reference & reference,
    const Scene & scene, Clock::time_point deadline);
  Solver solver_;
  std::vector<Pose> route_;
  std::optional<Solution> warm_;
  std::optional<double> station_;
  uint64_t track_version_=0;
};
}  // namespace mpc
