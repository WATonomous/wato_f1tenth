#include "local_planning/curves/curve_connection_generator.hpp"

#include "clothoids_backend.hpp"

#include <cmath>
#include <utility>

namespace local_planning
{

namespace
{

GeneratedConnection rejected(RejectReason reason)
{
  GeneratedConnection connection;
  connection.valid = false;
  connection.reject_reason = reason;
  return connection;
}

} // namespace

GeneratedConnection CurveConnectionGenerator::generate(const ConnectionRequest & request) const
{
  clothoids_backend::SolveResult solved =
    clothoids_backend::solveG2(request.start, request.terminal, config_.sample_spacing_m);

  if (!solved.converged) {
    return rejected(RejectReason::SOLVER_FAILED);
  }


  if (solved.arc_length_m > config_.max_arc_length_m) {
    return rejected(RejectReason::ARC_LENGTH_LIMIT);
  }
  if (solved.max_abs_curvature_inv_m > config_.max_curvature_inv_m) {
    return rejected(RejectReason::CURVATURE_LIMIT);
  }

  GeneratedConnection connection;

  connection.samples = std::move(solved.samples);

  // Jump two starts from what the solver produced, never from what was asked
  // for.  Speed rides through; the curve layer supplies geometry only.
  const CurveSample & last = connection.samples.back();
  connection.actual_terminal.x = last.x;
  connection.actual_terminal.y = last.y;
  connection.actual_terminal.heading = last.heading;
  connection.actual_terminal.curvature = last.curvature;
  connection.actual_terminal.speed = request.terminal.speed;

  connection.valid = true;
  connection.reject_reason = RejectReason::NONE;
  return connection;
}

} // namespace local_planning
