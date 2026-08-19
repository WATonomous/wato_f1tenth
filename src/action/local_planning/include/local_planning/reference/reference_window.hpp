#ifndef LOCAL_PLANNING_REFERENCE_REFERENCE_WINDOW_HPP
#define LOCAL_PLANNING_REFERENCE_REFERENCE_WINDOW_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/reference/raceline_reference.hpp"

#include <cstddef>
#include <vector>

namespace local_planning
{

// Per-cycle reference geometry on a uniform station grid; evaluated once, shared by all candidates.
class ReferenceWindow
{
public:
  // [s_start, s_start + length_m]; endpoints are exact (not snapped approximations).
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

  double sAt(std::size_t i) const {return s_start_ + spacing_ * static_cast<double>(i);}  // unwrapped

  std::size_t indexForS(double s) const;  // wrapping-aware nearest index, clamped

  const ReferenceGeometrySample & at(std::size_t i) const {return samples_[i];}

private:
  std::vector<ReferenceGeometrySample> samples_;
  const RacelineReference * reference_ = nullptr;
  double s_start_ = 0.0;
  double spacing_ = 0.0;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_REFERENCE_REFERENCE_WINDOW_HPP
