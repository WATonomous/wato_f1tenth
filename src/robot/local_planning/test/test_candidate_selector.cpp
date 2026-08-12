#include "local_planning/selection/candidate_selector.hpp"

#include <gtest/gtest.h>

namespace local_planning
{
namespace
{
ManeuverCandidate geometry(
  double distance = 0.0,
  double deviation = 0.0,
  double target_d = 0.0,
  bool uses_offset_tail = false)
{
  ManeuverCandidate result;
  result.maneuver_distance_m = distance;
  result.max_offset_deviation_m = deviation;
  result.target_d = target_d;
  result.uses_offset_tail = uses_offset_tail;
  return result;
}

EvaluatedCandidate evaluated(int index, CandidateSource source, CollisionStatus status, double time)
{
  EvaluatedCandidate result;
  result.candidate_index = index;
  result.source = source;
  result.collision.status = status;
  result.velocity_feasible = true;
  result.traversal_time_s = time;
  return result;
}
}  // namespace

TEST(CandidateSelector, OvertakeNormalBeatsFasterMarginal)
{
  const std::vector<ManeuverCandidate> pool(2);
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::OVERTAKE, CollisionStatus::SOFT_INFLATION, 1.0),
    evaluated(1, CandidateSource::OVERTAKE, CollisionStatus::FREE, 2.0)};
  EXPECT_EQ(CandidateSelector().selectOvertake(pool, candidates), 1);
}

TEST(CandidateSelector, OvertakeClothoidBeatsFasterTailAtEqualClearance)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(0.0, 0.0, 0.55, true), geometry()};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::OVERTAKE, CollisionStatus::FREE, 1.0),
    evaluated(1, CandidateSource::OVERTAKE, CollisionStatus::FREE, 2.0)};
  EXPECT_EQ(CandidateSelector().selectOvertake(pool, candidates), 1);
}

TEST(CandidateSelector, OvertakeTailBeatsClothoidWithWorseClearance)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(), geometry(0.0, 0.0, 0.55, true)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::OVERTAKE, CollisionStatus::SOFT_INFLATION, 1.0),
    evaluated(1, CandidateSource::OVERTAKE, CollisionStatus::FREE, 2.0)};
  EXPECT_EQ(CandidateSelector().selectOvertake(pool, candidates), 1);
}

TEST(CandidateSelector, PassNormalPreferredWinsImmediately)
{
  const std::vector<ManeuverCandidate> pool{geometry(), geometry(6.0, 0.01, 0.55)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::PASS_PREFERRED, CollisionStatus::FREE, 3.0),
    evaluated(1, CandidateSource::PASS_RECOVERY, CollisionStatus::FREE, 1.0)};
  EXPECT_EQ(CandidateSelector().selectPass(pool, candidates), 0);
}

TEST(CandidateSelector, PassMarginalPreferredSurvivesWithoutNormalRecovery)
{
  const std::vector<ManeuverCandidate> pool{geometry(), geometry(6.0, 0.01, 0.55)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::PASS_PREFERRED, CollisionStatus::SOFT_INFLATION, 3.0),
    evaluated(1, CandidateSource::PASS_RECOVERY, CollisionStatus::SOFT_INFLATION, 1.0)};
  EXPECT_EQ(CandidateSelector().selectPass(pool, candidates), 0);
}

TEST(CandidateSelector, PassRecoveryUsesLongestTierRegardlessOfInputOrder)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(1.0, 0.01, 0.55), geometry(6.0, 0.20, 0.75)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::PASS_RECOVERY, CollisionStatus::FREE, 1.0),
    evaluated(1, CandidateSource::PASS_RECOVERY, CollisionStatus::FREE, 2.0)};
  EXPECT_EQ(CandidateSelector().selectPass(pool, candidates), 1);
}

TEST(CandidateSelector, PassRecoveryUsesDeviationThenInwardThenTime)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(6.0, 0.20, 0.75), geometry(6.0, 0.10, 0.75),
    geometry(6.0, 0.10, 0.55), geometry(6.0, 0.10, 0.55)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::PASS_RECOVERY, CollisionStatus::FREE, 0.5),
    evaluated(1, CandidateSource::PASS_RECOVERY, CollisionStatus::FREE, 0.6),
    evaluated(2, CandidateSource::PASS_RECOVERY, CollisionStatus::FREE, 2.0),
    evaluated(3, CandidateSource::PASS_RECOVERY, CollisionStatus::FREE, 1.0)};
  EXPECT_EQ(CandidateSelector().selectPass(pool, candidates), 3);
}

TEST(CandidateSelector, MergeEqualTimePrefersLongerCompletion)
{
  const std::vector<ManeuverCandidate> pool{geometry(2.0), geometry(5.0)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::MERGE, CollisionStatus::FREE, 1.0),
    evaluated(1, CandidateSource::MERGE, CollisionStatus::FREE, 1.0)};
  EXPECT_EQ(CandidateSelector().selectMerge(pool, candidates), 1);
}

}  // namespace local_planning
