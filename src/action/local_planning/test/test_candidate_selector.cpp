#include "local_planning/selection/candidate_selector.hpp"

#include <gtest/gtest.h>

namespace local_planning
{
namespace
{
ManeuverCandidate geometry(
  double passing_d = 0.0,
  double terminal_d = 0.0,
  double distance = 0.0,
  bool uses_offset_tail = false)
{
  ManeuverCandidate result;
  result.passing_d = passing_d;
  result.terminal_d = terminal_d;
  result.maneuver_distance_m = distance;
  result.uses_offset_tail = uses_offset_tail;
  return result;
}

constexpr double kSoftInflationM = 0.18;

// clearance defaults to a value consistent with status, so tests that do not
// care about the margin do not have to state one: free sits beyond the
// inflation, soft sits halfway into it.
EvaluatedCandidate evaluated(
  int index,
  CandidateSource source,
  CollisionStatus status,
  double time,
  double clearance = -1.0)
{
  EvaluatedCandidate result;
  result.candidate_index = index;
  result.source = source;
  result.collision.status = status;
  result.collision.minimum_clearance_m = clearance >= 0.0 ?
    clearance :
    (status == CollisionStatus::FREE ? 2.0 * kSoftInflationM : 0.5 * kSoftInflationM);
  result.velocity_feasible = true;
  result.traversal_time_s = time;
  return result;
}

int select(
  const std::vector<ManeuverCandidate> & pool,
  const std::vector<EvaluatedCandidate> & candidates)
{
  return CandidateSelector(kSoftInflationM).select(pool, candidates);
}
}  // namespace

// --- Safety outranks everything, and it is graded ---------------------------

TEST(CandidateSelector, FreeBeatsFasterSoftInflation)
{
  const std::vector<ManeuverCandidate> pool(2);
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::OVERTAKE, CollisionStatus::SOFT_INFLATION, 1.0),
    evaluated(1, CandidateSource::OVERTAKE, CollisionStatus::FREE, 2.0)};
  EXPECT_EQ(select(pool, candidates), 1);
}

TEST(CandidateSelector, FreeAtLargerPassingOffsetBeatsSoftAtSmaller)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(0.55, 0.55), geometry(0.75, 0.75)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::OVERTAKE, CollisionStatus::SOFT_INFLATION, 1.0),
    evaluated(1, CandidateSource::OVERTAKE, CollisionStatus::FREE, 1.0)};
  EXPECT_EQ(select(pool, candidates), 1);
}

TEST(CandidateSelector, RejectsCollidingAndInfeasibleCandidates)
{
  const std::vector<ManeuverCandidate> pool(3);
  std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::OVERTAKE, CollisionStatus::COLLISION, 0.1),
    evaluated(1, CandidateSource::OVERTAKE, CollisionStatus::OUT_OF_GRID, 0.1),
    evaluated(2, CandidateSource::OVERTAKE, CollisionStatus::FREE, 0.1)};
  candidates[2].velocity_feasible = false;
  EXPECT_EQ(select(pool, candidates), -1);
}

// --- OVERTAKE ranks on geometry before time --------------------------------

TEST(CandidateSelector, OvertakePrefersSmallerPassingOffset)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(0.75, 0.75), geometry(0.55, 0.55)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::OVERTAKE, CollisionStatus::FREE, 1.0),
    evaluated(1, CandidateSource::OVERTAKE, CollisionStatus::FREE, 2.0)};
  // Slower, but it tucks closer to the line.
  EXPECT_EQ(select(pool, candidates), 1);
}

TEST(CandidateSelector, OvertakeRanksPassingOffsetByMagnitudeNotSign)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(-0.55, -0.55), geometry(0.75, 0.75)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::OVERTAKE, CollisionStatus::FREE, 2.0),
    evaluated(1, CandidateSource::OVERTAKE, CollisionStatus::FREE, 1.0)};
  EXPECT_EQ(select(pool, candidates), 0);
}

TEST(CandidateSelector, OvertakeBreaksPassingTiesOnTerminalOffset)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(0.55, 0.75), geometry(0.55, 0.55)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::OVERTAKE, CollisionStatus::FREE, 1.0),
    evaluated(1, CandidateSource::OVERTAKE, CollisionStatus::FREE, 2.0)};
  EXPECT_EQ(select(pool, candidates), 1);
}

TEST(CandidateSelector, OvertakeBreaksGeometryTiesOnTime)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(0.55, 0.55), geometry(0.55, 0.55)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::OVERTAKE, CollisionStatus::FREE, 2.0),
    evaluated(1, CandidateSource::OVERTAKE, CollisionStatus::FREE, 1.0)};
  EXPECT_EQ(select(pool, candidates), 1);
}

// The offset tail used to win on identity alone.  After the port it is an
// ordinary connection with d' = d'' = 0, so it competes on geometry like
// everything else -- and loses here on both offsets.
TEST(CandidateSelector, OvertakeTailNoLongerWinsForBeingATail)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(0.75, 0.75, 0.0, true), geometry(0.55, 0.55, 0.0, false)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::OVERTAKE, CollisionStatus::FREE, 1.0),
    evaluated(1, CandidateSource::OVERTAKE, CollisionStatus::FREE, 1.0)};
  EXPECT_EQ(select(pool, candidates), 1);
}

TEST(CandidateSelector, OvertakeKeepsFirstSeenOnAnExactTie)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(0.55, 0.55, 0.0, true), geometry(0.55, 0.55, 0.0, false)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::OVERTAKE, CollisionStatus::FREE, 1.0),
    evaluated(1, CandidateSource::OVERTAKE, CollisionStatus::FREE, 1.0)};
  EXPECT_EQ(select(pool, candidates), 0);
}

// --- PASS and MERGE are (safety, time) and nothing else --------------------

// The regression that mattered on track: PASS used to rank on time alone, so a
// wider offset could win for being marginally quicker.  Nothing then opposed the
// drift outward, and each cycle promoted the next offset out.  Geometry now
// leads, so the tightest clear offset wins even when it is slower.
TEST(CandidateSelector, PassPrefersTheTightestClearOffsetOverTheFasterWiderOne)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(0.55, 0.55, 6.0), geometry(0.75, 0.75, 3.0)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::PASS_PREFERRED, CollisionStatus::FREE, 2.0),
    evaluated(1, CandidateSource::PASS_RECOVERY, CollisionStatus::FREE, 1.0)};
  EXPECT_EQ(select(pool, candidates), 0);
}

// ...and it gives way when the tight offset is not actually clear.
TEST(CandidateSelector, PassFallsOutToTheNextOffsetWhenTheTightestIsNotFree)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(0.55, 0.55, 6.0), geometry(0.75, 0.75, 3.0)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::PASS_PREFERRED, CollisionStatus::SOFT_INFLATION, 2.0),
    evaluated(1, CandidateSource::PASS_RECOVERY, CollisionStatus::FREE, 3.0)};
  EXPECT_EQ(select(pool, candidates), 1);
}

// Soft inflation is graded, not binary: with no free candidate anywhere, the
// shallower intrusion wins even though it is wider and slower.
TEST(CandidateSelector, SoftInflationRanksOnActualClearanceNotJustStatus)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(0.55, 0.55, 6.0), geometry(0.75, 0.75, 3.0)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::PASS_PREFERRED, CollisionStatus::SOFT_INFLATION, 1.0, 0.02),
    evaluated(1, CandidateSource::PASS_RECOVERY, CollisionStatus::SOFT_INFLATION, 3.0, 0.16)};
  EXPECT_EQ(select(pool, candidates), 1);
}

// The inversion the old tier loop forbade outright: a recovery candidate with a
// longer transition can win, if it is faster.
TEST(CandidateSelector, PassLetsALongerTransitionWinWhenItIsFaster)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(0.55, 0.55, 1.0), geometry(0.55, 0.55, 6.0)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::PASS_RECOVERY, CollisionStatus::FREE, 2.0),
    evaluated(1, CandidateSource::PASS_RECOVERY, CollisionStatus::FREE, 1.0)};
  EXPECT_EQ(select(pool, candidates), 1);
}

TEST(CandidateSelector, PassBreaksGeometryTiesOnTimeRegardlessOfSource)
{
  const std::vector<ManeuverCandidate> pool{geometry(), geometry()};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::PASS_PREFERRED, CollisionStatus::FREE, 3.0),
    evaluated(1, CandidateSource::PASS_RECOVERY, CollisionStatus::FREE, 1.0)};
  EXPECT_EQ(select(pool, candidates), 1);
}

TEST(CandidateSelector, PassStillTakesSafetyOverTime)
{
  const std::vector<ManeuverCandidate> pool{geometry(), geometry()};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::PASS_RECOVERY, CollisionStatus::SOFT_INFLATION, 1.0),
    evaluated(1, CandidateSource::PASS_PREFERRED, CollisionStatus::FREE, 5.0)};
  EXPECT_EQ(select(pool, candidates), 1);
}

TEST(CandidateSelector, MergeTakesTheFastestAndIgnoresCompletionDistance)
{
  // Equal time; the old third key handed this to the longer completion.
  const std::vector<ManeuverCandidate> pool{
    geometry(0.0, 0.0, 1.0), geometry(0.0, 0.0, 6.0)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::MERGE, CollisionStatus::FREE, 1.0),
    evaluated(1, CandidateSource::MERGE, CollisionStatus::FREE, 1.0)};
  EXPECT_EQ(select(pool, candidates), 0);
}

TEST(CandidateSelector, MergePrefersTheFasterCompletion)
{
  const std::vector<ManeuverCandidate> pool{
    geometry(0.0, 0.0, 1.0), geometry(0.0, 0.0, 6.0)};
  const std::vector<EvaluatedCandidate> candidates{
    evaluated(0, CandidateSource::MERGE, CollisionStatus::FREE, 2.0),
    evaluated(1, CandidateSource::MERGE, CollisionStatus::FREE, 1.0)};
  EXPECT_EQ(select(pool, candidates), 1);
}

TEST(CandidateSelector, EmptyPoolSelectsNothing)
{
  const std::vector<ManeuverCandidate> pool;
  const std::vector<EvaluatedCandidate> candidates;
  EXPECT_EQ(select(pool, candidates), -1);
  EXPECT_EQ(select(pool, candidates), -1);
  EXPECT_EQ(select(pool, candidates), -1);
}

}  // namespace local_planning
