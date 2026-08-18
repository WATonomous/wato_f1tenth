#ifndef LOCAL_PLANNING_MANEUVERS_MANEUVER_BUILDER_HPP
#define LOCAL_PLANNING_MANEUVERS_MANEUVER_BUILDER_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/curves/frenet_connection_generator.hpp"
#include "local_planning/reference/raceline_reference.hpp"
#include "local_planning/reference/reference_window.hpp"

#include <vector>

namespace local_planning
{

using Path = std::vector<CurveSample>;

// Only values needed by later candidate selection live beside the geometry.
// Construction diagnostics intentionally remain inside the curve generator.
struct ManeuverCandidate
{
  Path path;
  // The two lateral offsets a candidate is characterised by.  These used to be
  // one `target_d` field carrying whichever of them the generating branch
  // happened to mean, which is most of why the old ranking was hard to read.
  //
  // passing_d is the offset at the end of the first leg -- beside the opponent,
  // the offset that has to fit.  terminal_d is the offset at the horizon.  For
  // a constant-offset tail they are the same value; for PASS and MERGE, which
  // have one commanded offset, both hold it.
  double passing_d = 0.0;
  double terminal_d = 0.0;
  double maneuver_distance_m = 0.0;
  // Worst |d| along the path, accumulated by the sampler as the path is built.
  // This is the overshoot measurement, and it is not derivable from passing_d
  // and terminal_d: the start boundary carries the measured d' and d'', and a
  // quintic leaving with those pointed outward swings past the offset it was
  // commanded to reach.  A displaced, yawed car is exactly when that happens,
  // which is also when track_bounds_rejected climbs.
  double max_abs_d_m = 0.0;
  // True only when a non-zero-length suffix holds a constant offset.  No longer
  // a ranking input -- after the port a tail is an ordinary Frenet connection
  // with d' = d'' = 0, not a different curve family -- but it stays on the wire
  // for bag-level diagnosis.
  bool uses_offset_tail = true;
};

struct ManeuverConfig
{
  double horizon_m = 4.0;

  std::vector<double> overtake_s_offsets_from_opponent_rear_m{0.0, 0.5};
  std::vector<double> passing_d_magnitudes_m{0.55, 0.75};
  std::vector<double> overtake_heading_offsets_rad{-0.15, 0.0, 0.15};
  std::vector<double> pass_transition_distances_m{4.0, 3.0, 1.0};
  std::vector<double> merge_completion_distances_m{0.5, 1.0, 2.0, 3.0, 4.0};

};

class ManeuverBuilder
{
public:
  ManeuverBuilder(
    const RacelineReference & reference,
    const FrenetConnectionGenerator & curve_generator,
    ManeuverConfig config,
    VehicleGeometry vehicle_geometry);

  std::vector<ManeuverCandidate> overtake(
    const BoundaryState & ego,
    double ego_s,
    double ego_d,
    double opponent_rear_s) const;
  std::vector<ManeuverCandidate> pass(
    const BoundaryState & ego, double ego_s, double ego_d) const;
  std::vector<ManeuverCandidate> recover(
    const BoundaryState & ego, double ego_s, double ego_d) const;
  // ego_d joins the signature because the start boundary is now Frenet: the
  // measured offset is an input to the connection, not something the world-frame
  // ego pose carried implicitly.
  std::vector<ManeuverCandidate> merge(
    const BoundaryState & ego, double ego_s, double ego_d) const;
  const ManeuverConfig & config() const {return config_;}

private:
  // Rebuilds the per-cycle reference table over [ego_s, ego_s + horizon_m].
  // Every entry point calls this before generating, and every leg of every
  // candidate then indexes the same table.  Unconditional: at ~61 spline
  // evaluations it is cheaper than reasoning about when the reference or the
  // station last changed, and it runs at most twice a cycle (PASS, then its
  // recovery family).
  // Also latches the speed the cycle's connections are shaped against, for the
  // same reason the window is cached here: both are per-cycle facts every leg
  // of every candidate needs, and threading them through connect() would put
  // them on the signature of every generation branch instead.
  bool prepareWindow(const BoundaryState & ego, double ego_s) const;

  // The measured car as a Frenet start boundary: its lateral offset, the slope
  // implied by its heading error, and the second derivative that continues the
  // curvature its current steering angle implies.
  bool startBoundary(
    const BoundaryState & ego, double ego_s, double ego_d, FrenetBoundary & result) const;
  // A commanded boundary on the reference: offset d at station s, turned
  // heading_offset away from the reference tangent, with d'' = 0.
  //
  // d'' = 0 is not a simplification, it is the whole of the old
  // BoundaryCurvature enum.  Setting d' = d'' = 0 in the curvature formula
  // gives kappa = kappa_ref / (1 - d kappa_ref), which is exactly what the
  // OFFSET mode used to request, and it is what makes a constant-offset tail C2
  // at the join by construction.
  bool boundary(
    double s, double d, double heading_offset, FrenetBoundary & result) const;
  // max_abs_d is raised to the worst |d| the appended leg reached, so a path
  // assembled from several legs carries the maximum over all of them.  It is
  // left untouched when the connection is rejected.
  bool connect(
    Path & path, const FrenetBoundary & start, const FrenetBoundary & end,
    double & max_abs_d) const;
  bool appendOffsetTail(
    Path & path, double start_s, double reference_distance_m, double d,
    double & max_abs_d) const;

  // A straight read of sample.d: the Newton refinement with its windowed-search
  // fallback is gone, because the offset is no longer something the path has to
  // be interrogated for.
  bool staysOnSide(const Path & path, int side) const;
  std::vector<double> offsets(int side) const;
  int sideOf(double d) const;

  const RacelineReference & reference_;
  const FrenetConnectionGenerator & curve_generator_;
  ManeuverConfig config_;
  VehicleGeometry vehicle_geometry_;
  // Mutable so the generation entry points stay const: these are per-cycle
  // caches of the reference and the measured speed, not part of the builder's
  // configuration.
  mutable ReferenceWindow window_;
  mutable double plan_speed_mps_ = 0.0;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_MANEUVERS_MANEUVER_BUILDER_HPP
