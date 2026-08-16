#include "local_planning/reference/reference_window.hpp"

#include <algorithm>
#include <cmath>

namespace local_planning
{
namespace
{
constexpr double kTolerance = 1e-9;
}  // namespace

bool ReferenceWindow::build(
  const RacelineReference & reference,
  double s_start,
  double length_m,
  double spacing_m)
{
  samples_.clear();
  reference_ = nullptr;
  if (!reference.valid() || !std::isfinite(s_start) ||
    !std::isfinite(length_m) || length_m <= 0.0 ||
    !std::isfinite(spacing_m) || spacing_m <= 0.0)
  {
    return false;
  }

  // Round the interval count up so the actual spacing is never coarser than
  // requested; the caller's spacing is the costmap resolution and planning must
  // not sample below it.
  const auto interval_count = static_cast<std::size_t>(
    std::max(1.0, std::ceil(length_m / spacing_m - kTolerance)));
  s_start_ = s_start;
  spacing_ = length_m / static_cast<double>(interval_count);
  reference_ = &reference;

  samples_.resize(interval_count + 1);
  for (std::size_t i = 0; i <= interval_count; ++i) {
    samples_[i] = reference.sampleAtS(s_start_ + spacing_ * static_cast<double>(i));
  }
  return true;
}

std::size_t ReferenceWindow::indexForS(double s) const
{
  if (samples_.empty() || reference_ == nullptr || spacing_ <= 0.0) {
    return 0;
  }
  const double progress = reference_->deltaS(s_start_, s);
  const double index = std::round(progress / spacing_);
  if (!(index > 0.0)) {
    return 0;
  }
  return std::min(static_cast<std::size_t>(index), samples_.size() - 1);
}

}  // namespace local_planning
