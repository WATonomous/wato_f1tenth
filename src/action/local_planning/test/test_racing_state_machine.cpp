#include <gtest/gtest.h>

#include "local_planning/state/racing_state_machine.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadius = 20.0;
constexpr double kResolution = 0.05;

// A detected face is good to roughly two cells: one scan step, one straddle.
constexpr double kFaceTolerance = 0.12;

std::vector<Point> circleLine(double radius, int count)
{
  std::vector<Point> points;
  points.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(count);
    points.emplace_back(radius * std::cos(angle), radius * std::sin(angle), 3.0);
  }
  return points;
}

RacelineReference makeReference()
{
  RacelineReference reference;
  EXPECT_TRUE(reference.setRacingLine(circleLine(kRadius, 400)));
  return reference;
}

StateMachineConfig defaultConfig()
{
  return StateMachineConfig{};
}

Odometry egoAt(
  const RacelineReference & reference,
  double s,
  double d,
  double heading_offset = 0.0)
{
  const ReferenceGeometrySample sample = reference.sampleAtS(s);
  Odometry ego;
  ego.position = reference.toCartesian(s, d);
  ego.heading = sample.heading + heading_offset;
  ego.velocity = sample.velocity;
  return ego;
}

// The detector's reach is bounded by this rectangle, not by a tuned distance.
OccupancyGrid gridAround(const Point & centre, double half_extent_m)
{
  const int cells = static_cast<int>(std::round(2.0 * half_extent_m / kResolution));
  OccupancyGrid grid;
  grid.width = cells;
  grid.height = cells;
  grid.resolution = kResolution;
  grid.origin = Point(centre.x - half_extent_m, centre.y - half_extent_m);
  grid.data.assign(static_cast<std::size_t>(cells) * static_cast<std::size_t>(cells), 0);
  return grid;
}

// Fills the corridor over [s_start, s_end], putting the near face at s_start.
void stampOpponent(
  OccupancyGrid & grid,
  const RacelineReference & reference,
  double s_start,
  double s_end,
  double d_half = 0.10)
{
  const double step = 0.5 * kResolution;
  for (double s = s_start; s <= s_end + 1e-9; s += step) {
    for (double d = -d_half; d <= d_half + 1e-9; d += step) {
      const Point p = reference.toCartesian(s, d);
      const int col = static_cast<int>(std::floor((p.x - grid.origin.x) / grid.resolution));
      const int row = static_cast<int>(std::floor((p.y - grid.origin.y) / grid.resolution));
      if (col < 0 || col >= grid.width || row < 0 || row >= grid.height) {
        continue;
      }
      grid.data[static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.width) +
        static_cast<std::size_t>(col)] = 100;
    }
  }
}

// Intent is a pure function of (ego, grid), so every case is one update.
struct Cycle
{
  RacelineReference reference;
  OccupancyGrid grid;
  Odometry ego;
  TacticalState state;
};

Cycle runOnce(
  double ego_s, double ego_d, double heading_offset, double opponent_start_s,
  double opponent_end_s, bool with_opponent, StateMachineConfig config = defaultConfig())
{
  Cycle cycle;
  cycle.reference = makeReference();
  cycle.ego = egoAt(cycle.reference, ego_s, ego_d, heading_offset);
  cycle.grid = gridAround(cycle.ego.position, 8.0);
  if (with_opponent) {
    stampOpponent(cycle.grid, cycle.reference, opponent_start_s, opponent_end_s);
  }

  RacingStateMachine machine(cycle.reference, config);
  machine.update(cycle.ego, cycle.grid);
  cycle.state = machine.state();
  return cycle;
}

Cycle runWithoutOpponent(double ego_s, double ego_d, double heading_offset = 0.0)
{
  return runOnce(ego_s, ego_d, heading_offset, 0.0, 0.0, false);
}

// Gap is to the opponent's near face; positive is ahead.
Cycle runWithOpponentAtGap(double ego_s, double ego_d, double gap_m, double length_m = 1.0)
{
  const double start_s = gap_m >= 0.0 ? ego_s + gap_m : ego_s + gap_m - length_m;
  return runOnce(ego_s, ego_d, 0.0, start_s, start_s + length_m, true);
}

TEST(RacingStateMachine, NoOpponentOnTheLineFollows)
{
  const Cycle cycle = runWithoutOpponent(2.0, 0.0);
  EXPECT_FALSE(cycle.state.opponent.detected);
  EXPECT_EQ(cycle.state.relative_position, RelativePosition::NONE);
  EXPECT_EQ(cycle.state.intent, PlannerIntent::FOLLOW_RACING_LINE);
}

// PRD 5: an offset car is never handed to the global follower.
TEST(RacingStateMachine, NoOpponentOffTheLineMerges)
{
  const Cycle cycle = runWithoutOpponent(2.0, 0.60);
  EXPECT_FALSE(cycle.state.opponent.detected);
  EXPECT_EQ(cycle.state.intent, PlannerIntent::MERGE);
}

TEST(RacingStateMachine, BehindInsideStartGapOvertakes)
{
  const Cycle cycle = runWithOpponentAtGap(2.0, 0.0, 2.0);
  ASSERT_TRUE(cycle.state.opponent.detected);
  EXPECT_EQ(cycle.state.relative_position, RelativePosition::BEHIND);
  EXPECT_NEAR(cycle.state.opponent.gap_m, 2.0, kFaceTolerance);
  EXPECT_EQ(cycle.state.intent, PlannerIntent::OVERTAKE);
}

// Beyond the start gap an anchored candidate falls outside the horizon.
TEST(RacingStateMachine, BehindBeyondStartGapOnTheLineFollows)
{
  const Cycle cycle = runWithOpponentAtGap(2.0, 0.0, 5.0);
  ASSERT_TRUE(cycle.state.opponent.detected);
  EXPECT_EQ(cycle.state.relative_position, RelativePosition::BEHIND);
  EXPECT_EQ(cycle.state.intent, PlannerIntent::FOLLOW_RACING_LINE);
}

TEST(RacingStateMachine, BehindBeyondStartGapOffTheLineMerges)
{
  const Cycle cycle = runWithOpponentAtGap(2.0, 0.60, 5.0);
  ASSERT_TRUE(cycle.state.opponent.detected);
  EXPECT_EQ(cycle.state.relative_position, RelativePosition::BEHIND);
  EXPECT_EQ(cycle.state.intent, PlannerIntent::MERGE);
}

TEST(RacingStateMachine, OverlappingPasses)
{
  const Cycle cycle = runWithOpponentAtGap(2.0, 0.50, 0.20);
  ASSERT_TRUE(cycle.state.opponent.detected);
  EXPECT_EQ(cycle.state.relative_position, RelativePosition::OVERLAPPING);
  EXPECT_EQ(cycle.state.intent, PlannerIntent::PASS);
}

// Same intent as OVERLAPPING on purpose: no OVERTAKE -> MERGE jump mid-pass.
TEST(RacingStateMachine, AheadNotClearAlsoPasses)
{
  const Cycle cycle = runWithOpponentAtGap(2.0, 0.50, -0.90);
  ASSERT_TRUE(cycle.state.opponent.detected);
  EXPECT_EQ(cycle.state.relative_position, RelativePosition::AHEAD_NOT_CLEAR);
  EXPECT_EQ(cycle.state.intent, PlannerIntent::PASS);
}

TEST(RacingStateMachine, AheadAndClearOffTheLineMerges)
{
  const Cycle cycle = runWithOpponentAtGap(2.0, 0.50, -2.00);
  ASSERT_TRUE(cycle.state.opponent.detected);
  EXPECT_EQ(cycle.state.relative_position, RelativePosition::AHEAD_AND_CLEAR);
  EXPECT_EQ(cycle.state.intent, PlannerIntent::MERGE);
}

// The pass ended on the line: there is no displacement left to merge away, so
// MERGE here would plan a maneuver back onto the station ego already occupies.
TEST(RacingStateMachine, AheadAndClearOnTheLineFollows)
{
  const Cycle cycle = runWithOpponentAtGap(2.0, 0.0, -2.00);
  ASSERT_TRUE(cycle.state.opponent.detected);
  EXPECT_EQ(cycle.state.relative_position, RelativePosition::AHEAD_AND_CLEAR);
  EXPECT_TRUE(cycle.state.raceline_compatible);
  EXPECT_EQ(cycle.state.intent, PlannerIntent::FOLLOW_RACING_LINE);
}

// Brackets, not exact boundaries: at a one-cell scan step, asserting to within
// a cell of 0.80 / 1.00 would measure the grid resolution, not the classifier.
// The two boundaries are only 0.20 apart, so AHEAD_NOT_CLEAR is bracketed at
// its midpoint.
TEST(RacingStateMachine, ClassifyBracketsEachBoundary)
{
  EXPECT_EQ(
    runWithOpponentAtGap(2.0, 0.0, 1.00).state.relative_position,
    RelativePosition::BEHIND);
  EXPECT_EQ(
    runWithOpponentAtGap(2.0, 0.0, 0.60).state.relative_position,
    RelativePosition::OVERLAPPING);
  EXPECT_EQ(
    runWithOpponentAtGap(2.0, 0.0, -0.60).state.relative_position,
    RelativePosition::OVERLAPPING);
  EXPECT_EQ(
    runWithOpponentAtGap(2.0, 0.0, -0.90).state.relative_position,
    RelativePosition::AHEAD_NOT_CLEAR);
  EXPECT_EQ(
    runWithOpponentAtGap(2.0, 0.0, -1.30).state.relative_position,
    RelativePosition::AHEAD_AND_CLEAR);
  EXPECT_EQ(
    runWithOpponentAtGap(2.0, 0.0, -1.80).state.relative_position,
    RelativePosition::AHEAD_AND_CLEAR);
}

TEST(RacingStateMachine, LateralToleranceGatesTheHandoff)
{
  const StateMachineConfig config = defaultConfig();
  EXPECT_EQ(
    runWithoutOpponent(2.0, config.compat_lateral_m - 0.05).state.intent,
    PlannerIntent::FOLLOW_RACING_LINE);
  EXPECT_EQ(
    runWithoutOpponent(2.0, config.compat_lateral_m + 0.05).state.intent,
    PlannerIntent::MERGE);
}

// Only a wrong-way heading gates the handoff; see local_planner.yaml.
TEST(RacingStateMachine, WrongWayHeadingGatesTheHandoff)
{
  const StateMachineConfig config = defaultConfig();
  EXPECT_EQ(
    runWithoutOpponent(2.0, 0.0, config.compat_heading_rad - 0.05).state.intent,
    PlannerIntent::FOLLOW_RACING_LINE);
  EXPECT_EQ(
    runWithoutOpponent(2.0, 0.0, config.compat_heading_rad + 0.05).state.intent,
    PlannerIntent::MERGE);
}

// The flapping this threshold was widened to kill: a car sitting inside the
// lateral deadband and yawing across the old 0.30 rad limit was called
// incompatible on every other cycle, so the intent oscillated at the yaw
// frequency while ego_d never moved. Both signs, because the wobble is
// two-sided and a one-sided test would pass against a botched wrapAngle.
TEST(RacingStateMachine, YawWobbleOnTheLineDoesNotMerge)
{
  for (const double heading_offset : {-0.44, -0.30, 0.30, 0.44}) {
    const Cycle cycle = runWithoutOpponent(2.0, -0.05, heading_offset);
    EXPECT_EQ(cycle.state.intent, PlannerIntent::FOLLOW_RACING_LINE)
      << "heading_offset " << heading_offset;
    EXPECT_TRUE(cycle.state.raceline_compatible)
      << "heading_offset " << heading_offset;
  }
}

// What deltaS exists for: a naive subtraction reports nearly a full lap here.
TEST(RacingStateMachine, GapIsWrapAwareAcrossTheStartLine)
{
  const RacelineReference reference = makeReference();
  const double length = reference.totalLength();

  const Odometry ego = egoAt(reference, length - 0.30, 0.0);
  OccupancyGrid grid = gridAround(ego.position, 8.0);
  stampOpponent(grid, reference, 0.20, 1.20);

  RacingStateMachine machine(reference, defaultConfig());
  machine.update(ego, grid);
  const TacticalState & state = machine.state();

  ASSERT_TRUE(state.opponent.detected);
  EXPECT_NEAR(state.opponent.gap_m, 0.50, kFaceTolerance);
  EXPECT_EQ(state.relative_position, RelativePosition::OVERLAPPING);
  EXPECT_EQ(state.intent, PlannerIntent::PASS);
}

// Phase 6 anchors OVERTAKE here, so it must be the near face -- not the
// centroid, not the far edge.
TEST(RacingStateMachine, DetectorReportsTheNearFace)
{
  const RacelineReference reference = makeReference();
  const double ego_s = 2.0;

  const Odometry ego = egoAt(reference, ego_s, 0.0);
  OccupancyGrid grid = gridAround(ego.position, 8.0);
  stampOpponent(grid, reference, ego_s + 2.0, ego_s + 3.0);

  RacingStateMachine machine(reference, defaultConfig());
  machine.update(ego, grid);
  const TacticalState & state = machine.state();

  ASSERT_TRUE(state.opponent.detected);
  EXPECT_NEAR(state.opponent.s, reference.wrapS(ego_s + 2.0), kFaceTolerance);
  EXPECT_NEAR(state.opponent.gap_m, 2.0, kFaceTolerance);
}

// Phase 6 reads these instead of projecting again; a silent disagreement means
// building candidates from a different station than the intent was chosen at.
TEST(RacingStateMachine, EgoProjectionIsAUsableOutput)
{
  const RacelineReference reference = makeReference();
  const Odometry ego = egoAt(reference, 2.0, 0.25);
  const OccupancyGrid grid = gridAround(ego.position, 8.0);

  RacingStateMachine machine(reference, defaultConfig());
  machine.update(ego, grid);

  // A fresh machine seeds from zero, so this is the same call it made.
  const Projection expected = reference.project(ego.position, ego.heading, 0.0);
  EXPECT_NEAR(machine.state().ego_s, expected.s, 1e-9);
  EXPECT_NEAR(machine.state().ego_d, expected.d, 1e-9);
  EXPECT_NEAR(machine.state().ego_d, 0.25, 1e-3);
}

TEST(RacingStateMachine, InvalidReferenceProducesTheDefaultState)
{
  const RacelineReference reference;   // no racing line set
  ASSERT_FALSE(reference.valid());

  RacingStateMachine machine(reference, defaultConfig());
  machine.update(Odometry{}, OccupancyGrid{});

  EXPECT_FALSE(machine.state().opponent.detected);
  EXPECT_EQ(machine.state().relative_position, RelativePosition::NONE);
  EXPECT_EQ(machine.state().intent, PlannerIntent::FOLLOW_RACING_LINE);
}

TEST(RacingStateMachine, EmptyGridDetectsNothing)
{
  const RacelineReference reference = makeReference();
  const Odometry ego = egoAt(reference, 2.0, 0.0);

  RacingStateMachine machine(reference, defaultConfig());
  machine.update(ego, OccupancyGrid{});

  EXPECT_FALSE(machine.state().opponent.detected);
  EXPECT_EQ(machine.state().intent, PlannerIntent::FOLLOW_RACING_LINE);
}

} // namespace
} // namespace local_planning
