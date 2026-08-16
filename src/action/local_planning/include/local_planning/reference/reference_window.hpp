#ifndef LOCAL_PLANNING_REFERENCE_REFERENCE_WINDOW_HPP
#define LOCAL_PLANNING_REFERENCE_REFERENCE_WINDOW_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/reference/raceline_reference.hpp"

#include <cstddef>
#include <vector>

namespace local_planning
{

// One cycle's worth of reference geometry on a uniform station grid, evaluated
// once and shared by every candidate.
//
// This is where the compute win of the port actually lands.  Under the clothoid
// family each candidate walked the reference independently: its own binary
// search into the spline and its own polynomial evaluation per sample, times
// however many candidates the enumeration produced.  Every Frenet candidate is
// sampled on this same grid, so the reference is evaluated once per cycle -- 61
// samples at the current 6 m horizon and 0.1 m spacing -- and per-candidate cost
// falls to Horner evaluations plus normal-offset arithmetic on cached data.
//
// Maneuver stations snap to grid indices.  The spacing is the costmap
// resolution, so the snap is below the resolution of the thing the paths are
// checked against, and it is what lets legs of different maneuvers share one
// table.
class ReferenceWindow
{
public:
  // Covers [s_start, s_start + length_m] inclusive, at the largest spacing not
  // exceeding spacing_m that divides length_m evenly.  Index 0 is exactly
  // s_start and the last index is exactly s_start + length_m, so neither the
  // ego boundary nor the horizon boundary is ever a snapped approximation.
  bool build(
    const RacelineReference & reference,
    double s_start,
    double length_m,
    double spacing_m);

  bool valid() const {return !samples_.empty();}
  std::size_t size() const {return samples_.size();}
  double startS() const {return s_start_;}
  double spacingM() const {return spacing_;}
  double lengthM() const {return spacing_ * static_cast<double>(samples_.size() - 1);}

  // Absolute (unwrapped) station of grid index i.  Use at(i).s_wrapped for the
  // value that goes on a CurveSample.
  double sAt(std::size_t i) const {return s_start_ + spacing_ * static_cast<double>(i);}

  // Nearest grid index to s, clamped to the window.  Wrapping-aware: the
  // distance is measured with deltaS, so a station just past the loop seam
  // still lands where it should.
  std::size_t indexForS(double s) const;

  const ReferenceGeometrySample & at(std::size_t i) const {return samples_[i];}

private:
  std::vector<ReferenceGeometrySample> samples_;
  const RacelineReference * reference_ = nullptr;
  double s_start_ = 0.0;
  double spacing_ = 0.0;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_REFERENCE_REFERENCE_WINDOW_HPP
