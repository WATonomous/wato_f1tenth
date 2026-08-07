#include "local_planning/selection/candidate_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_planning
{
namespace
{
constexpr double kTolerance = 1e-6;

bool valid(const EvaluatedCandidate & candidate)
{
  return candidate.candidate_index >= 0 && candidate.velocity_feasible &&
         (candidate.collision.status == CollisionStatus::FREE ||
         candidate.collision.status == CollisionStatus::SOFT_INFLATION);
}

int clearanceRank(const EvaluatedCandidate & candidate)
{
  return candidate.collision.status == CollisionStatus::FREE ? 0 : 1;
}
}  // namespace

int CandidateSelector::selectOvertake(
  const std::vector<ManeuverCandidate> & pool,
  const std::vector<EvaluatedCandidate> & evaluated) const
{
  (void)pool;
  const EvaluatedCandidate * best = nullptr;
  for (const auto & candidate : evaluated) {
    if (valid(candidate) &&
      (!best || clearanceRank(candidate) < clearanceRank(*best) ||
      (clearanceRank(candidate) == clearanceRank(*best) &&
      candidate.traversal_time_s < best->traversal_time_s)))
    {
      best = &candidate;
    }
  }
  return best ? best->candidate_index : -1;
}

int CandidateSelector::selectPass(
  const std::vector<ManeuverCandidate> & pool,
  const std::vector<EvaluatedCandidate> & evaluated) const
{
  const EvaluatedCandidate * preferred = nullptr;
  for (const auto & candidate : evaluated) {
    if (candidate.source == CandidateSource::PASS_PREFERRED && valid(candidate)) {
      preferred = &candidate;
      if (candidate.collision.status == CollisionStatus::FREE) {
        return candidate.candidate_index;
      }
      break;
    }
  }

  std::vector<double> tiers;
  for (const auto & candidate : evaluated) {
    if (candidate.source == CandidateSource::PASS_RECOVERY && valid(candidate)) {
      tiers.push_back(pool.at(
          static_cast<std::size_t>(candidate.candidate_index)).maneuver_distance_m);
    }
  }
  std::sort(tiers.begin(), tiers.end(), std::greater<double>());
  tiers.erase(std::unique(tiers.begin(), tiers.end(), [](double a, double b) {
      return std::abs(a - b) <= kTolerance;
    }), tiers.end());

  for (const double tier : tiers) {
    const EvaluatedCandidate * best = nullptr;
    for (const auto & candidate : evaluated) {
      if (candidate.source != CandidateSource::PASS_RECOVERY || !valid(candidate)) {continue;}
      const auto & geometry = pool.at(static_cast<std::size_t>(candidate.candidate_index));
      if (std::abs(geometry.maneuver_distance_m - tier) > kTolerance) {continue;}
      if (!best) {best = &candidate; continue;}
      const auto & best_geometry = pool.at(static_cast<std::size_t>(best->candidate_index));
      if (clearanceRank(candidate) != clearanceRank(*best)) {
        if (clearanceRank(candidate) < clearanceRank(*best)) {best = &candidate;}
      } else if (geometry.max_offset_deviation_m <
        best_geometry.max_offset_deviation_m - kTolerance)
      {
        best = &candidate;
      } else if (std::abs(geometry.max_offset_deviation_m -
        best_geometry.max_offset_deviation_m) <= kTolerance &&
        std::abs(geometry.target_d) < std::abs(best_geometry.target_d) - kTolerance)
      {
        best = &candidate;
      } else if (std::abs(geometry.max_offset_deviation_m -
        best_geometry.max_offset_deviation_m) <= kTolerance &&
        std::abs(std::abs(geometry.target_d) - std::abs(best_geometry.target_d)) <= kTolerance &&
        candidate.traversal_time_s < best->traversal_time_s)
      {
        best = &candidate;
      }
    }

    if (!best) {continue;}
    if (best->collision.status == CollisionStatus::FREE) {
      return best->candidate_index;
    }
    if (!preferred) {
      return best->candidate_index;
    }
  }
  return preferred ? preferred->candidate_index : -1;
}

int CandidateSelector::selectMerge(
  const std::vector<ManeuverCandidate> & pool,
  const std::vector<EvaluatedCandidate> & evaluated) const
{
  const EvaluatedCandidate * best = nullptr;
  for (const auto & candidate : evaluated) {
    if (!valid(candidate)) {continue;}
    if (!best) {best = &candidate; continue;}
    if (clearanceRank(candidate) < clearanceRank(*best)) {
      best = &candidate;
      continue;
    }
    if (clearanceRank(candidate) > clearanceRank(*best)) {continue;}
    if (candidate.traversal_time_s < best->traversal_time_s - kTolerance) {
      best = &candidate;
    } else if (std::abs(candidate.traversal_time_s - best->traversal_time_s) <= kTolerance &&
      pool.at(static_cast<std::size_t>(candidate.candidate_index)).maneuver_distance_m >
      pool.at(static_cast<std::size_t>(best->candidate_index)).maneuver_distance_m)
    {
      best = &candidate;
    }
  }
  return best ? best->candidate_index : -1;
}

}  // namespace local_planning
