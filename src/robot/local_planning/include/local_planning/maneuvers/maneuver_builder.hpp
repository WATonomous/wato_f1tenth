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

struct ManeuverCandidate
{
  Path path;
  double passing_d = 0.0;   // lateral offset at pass point (beside opponent)
  double terminal_d = 0.0;    // lateral offset at horizon
  double maneuver_distance_m = 0.0;
  double max_abs_d_m = 0.0;   // worst |d| along path; can exceed terminal_d
  bool uses_offset_tail = true;  // diagnostic: path ends with constant-offset tail
};

struct ManeuverConfig
{
  double horizon_m = 4.0;

  std::vector<double> overtake_s_offsets_from_opponent_rear_m{0.0, 0.5};
  std::vector<double> passing_d_magnitudes_m{0.55, 0.75};
  std::vector<double> overtake_heading_offsets_rad{-0.15, 0.0, 0.15};  // must include 0.0
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
  std::vector<ManeuverCandidate> merge(
    const BoundaryState & ego, double ego_s, double ego_d) const;
  const ManeuverConfig & config() const {return config_;}

private:
  bool prepareWindow(const BoundaryState & ego, double ego_s) const;  // per-cycle ref grid + plan speed

  bool startBoundary(
    const BoundaryState & ego, double ego_s, double ego_d, FrenetBoundary & result) const;
  bool boundary(
    double s, double d, double heading_offset, FrenetBoundary & result) const;  // d'' = 0
  bool connect(
    Path & path, const FrenetBoundary & start, const FrenetBoundary & end,
    double & max_abs_d) const;
  bool appendOffsetTail(
    Path & path, double start_s, double reference_distance_m, double d,
    double & max_abs_d) const;

  bool staysOnSide(const Path & path, int side) const;
  std::vector<double> offsets(int side) const;
  int sideOf(double d) const;

  struct Waypoint
  {
    double s = 0.0;
    double d = 0.0;
    double heading_offset = 0.0;
  };
  struct ChainSpec
  {
    std::vector<Waypoint> waypoints;
    double passing_d = 0.0;
    double terminal_d = 0.0;
    double maneuver_distance_m = 0.0;
    bool uses_offset_tail = false;
    int side = 0;
    bool append_tail = false;
    double tail_start_s = 0.0;
    double tail_distance_m = 0.0;
    double tail_d = 0.0;
  };

  bool beginGeneration(
    const BoundaryState & ego, double ego_s, double ego_d, FrenetBoundary & start) const;
  std::vector<ManeuverCandidate> sampleChains(
    const FrenetBoundary & start, const std::vector<ChainSpec> & chains) const;

  const RacelineReference & reference_;
  const FrenetConnectionGenerator & curve_generator_;
  ManeuverConfig config_;
  VehicleGeometry vehicle_geometry_;
  mutable ReferenceWindow window_;       // per-cycle cache; entry points stay const
  mutable double plan_speed_mps_ = 0.0;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_MANEUVERS_MANEUVER_BUILDER_HPP
