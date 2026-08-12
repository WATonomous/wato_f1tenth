#ifndef LOCAL_PLANNING_MANEUVERS_MANEUVER_BUILDER_HPP
#define LOCAL_PLANNING_MANEUVERS_MANEUVER_BUILDER_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/curves/curve_connection_generator.hpp"
#include "local_planning/reference/raceline_reference.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace local_planning
{

using Path = std::vector<CurveSample>;

// Only values needed by later candidate selection live beside the geometry.
// Construction diagnostics intentionally remain inside the curve generator.
struct ManeuverCandidate
{
  Path path;
  double target_d = 0.0;
  double maneuver_distance_m = 0.0;
  double max_offset_deviation_m = 0.0;
  // True only when a non-zero-length suffix samples the exact constant-offset
  // reference.  The selector uses this to keep the existing clothoid family
  // preferred when clearance is equal, and ROS exposes the selected value for
  // bag-level diagnosis.
  bool uses_offset_tail = false;
};

struct ManeuverConfig
{
  double horizon_m = 6.0;

  std::vector<double> overtake_s_offsets_from_opponent_rear_m{0.0, 0.5, 1.0};
  std::vector<double> passing_d_magnitudes_m{0.55, 0.75};
  std::vector<double> overtake_heading_offsets_rad{-0.15, 0.0, 0.15};
  std::vector<double> pass_transition_distances_m{6.0, 3.0, 1.0};
  std::vector<double> merge_completion_distances_m{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

};

class ManeuverBuilder
{
public:
  ManeuverBuilder(
    const RacelineReference & reference,
    const CurveConnectionGenerator & curve_generator,
    ManeuverConfig config,
    VehicleGeometry vehicle_geometry);

  std::vector<ManeuverCandidate> overtake(
    const BoundaryState & ego,
    double ego_s,
    double ego_d,
    double opponent_rear_s,
    SustainableBounds bounds = SustainableBounds::unbounded(),
    uint32_t * track_bounds_rejected = nullptr) const;
  std::vector<ManeuverCandidate> pass(
    const BoundaryState & ego, double ego_s, double ego_d,
    SustainableBounds bounds = SustainableBounds::unbounded(),
    uint32_t * track_bounds_rejected = nullptr) const;
  std::vector<ManeuverCandidate> recover(
    const BoundaryState & ego, double ego_s, double ego_d,
    SustainableBounds bounds = SustainableBounds::unbounded(),
    uint32_t * track_bounds_rejected = nullptr) const;
  std::vector<ManeuverCandidate> merge(
    const BoundaryState & ego, double ego_s,
    SustainableBounds bounds = SustainableBounds::unbounded(),
    uint32_t * track_bounds_rejected = nullptr) const;
  const ManeuverConfig & config() const {return config_;}

  // How often the cheap station-hinted offset failed and had to fall back to a
  // windowed search.  This is the difference between the fast path and the slow
  // one, and nothing else in the output distinguishes them, so it is counted
  // and reported rather than left to be inferred from a latency spike.  Reset
  // per cycle by the caller.
  struct StationHintStats
  {
    uint64_t samples = 0;
    uint64_t fallbacks = 0;
  };
  StationHintStats stationHintStats() const {return station_hint_stats_;}
  void resetStationHintStats() const {station_hint_stats_ = {};}

private:
  enum class BoundaryCurvature
  {
    REFERENCE,
    OFFSET
  };

  bool boundary(
    double s,
    double d,
    double heading_offset,
    BoundaryState & result,
    BoundaryCurvature curvature = BoundaryCurvature::OFFSET) const;
  bool connect(
    Path & path,
    const BoundaryState & start,
    const BoundaryState & end,
    double start_raceline_s,
    double end_raceline_s,
    BoundaryState * actual_end = nullptr) const;
  bool appendReferenceCurve(
    Path & path, double start_s, double reference_distance_m, double d) const;

  struct SideCheck
  {
    bool stays_on_side = false;
    // Only meaningful when stays_on_side; the sweep returns early otherwise.
    double max_offset_deviation_m = 0.0;
  };

  // One sweep answering both questions the old staysOnSide() and
  // maximumOffsetDeviation() asked separately.  They walked the same path in
  // the same order and each computed the same lateral offset per sample, so
  // splitting them doubled the cost of the dominant term in PASS.
  SideCheck sideAndDeviation(
    const Path & path,
    int side,
    bool allow_start_center,
    double target_d) const;
  std::vector<double> offsets(
    int side, SustainableBounds bounds,
    uint32_t * track_bounds_rejected = nullptr) const;
  std::optional<double> preferredOffset(
    double ego_d, SustainableBounds bounds,
    uint32_t * track_bounds_rejected = nullptr) const;
  int sideOf(double d) const;

  const RacelineReference & reference_;
  const CurveConnectionGenerator & curve_generator_;
  ManeuverConfig config_;
  VehicleGeometry vehicle_geometry_;
  // Mutable so the generation entry points stay const: this is diagnostics
  // about how the answer was reached, not part of the answer.
  mutable StationHintStats station_hint_stats_;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_MANEUVERS_MANEUVER_BUILDER_HPP
