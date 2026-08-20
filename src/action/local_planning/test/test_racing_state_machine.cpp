#include <gtest/gtest.h>

#include "local_planning/state/racing_state_machine.hpp"
#include "local_planning/core/geometry.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace local_planning
{
namespace
{

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
  double d_half = 0.10,
  int8_t value = 100)
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
        static_cast<std::size_t>(col)] = value;
    }
  }
}

void stampOffsetWall(
  OccupancyGrid & grid,
  const RacelineReference & reference,
  double s_start,
  double s_end,
  double d)
{
  const double step = 0.5 * kResolution;
  for (double s = s_start; s <= s_end + 1e-9; s += step) {
    const Point p = reference.toCartesian(s, d);
    const int col = static_cast<int>(std::floor((p.x - grid.origin.x) / grid.resolution));
    const int row = static_cast<int>(std::floor((p.y - grid.origin.y) / grid.resolution));
    if (col >= 0 && col < grid.width && row >= 0 && row < grid.height) {
      grid.data[static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.width) +
        static_cast<std::size_t>(col)] = 100;
    }
  }
}

// Settle through the configured debounce; the focused tests below inspect the
// intermediate evidence explicitly.
struct Cycle
{
  RacelineReference reference;
  OccupancyGrid grid;
  Odometry ego;
  TacticalState state;
};

Cycle runOnce(
  double ego_s, double ego_d, double heading_offset, double opponent_start_s,
  double opponent_end_s, bool with_opponent, StateMachineConfig config = defaultConfig(),
  VehicleGeometry vehicle_geometry = VehicleGeometry{}, GridPolicy grid_policy = GridPolicy{})
{
  Cycle cycle;
  cycle.reference = makeReference();
  cycle.ego = egoAt(cycle.reference, ego_s, ego_d, heading_offset);
  cycle.grid = gridAround(cycle.ego.position, 8.0);
  if (with_opponent) {
    stampOpponent(cycle.grid, cycle.reference, opponent_start_s, opponent_end_s);
  }

  RacingStateMachine machine(cycle.reference, config, vehicle_geometry, grid_policy);
  for (int i = 0; i < 5; ++i) {
    machine.update(cycle.ego, cycle.grid);
    machine.reportMergeProbe(true);
  }
  cycle.state = machine.state();
  return cycle;
}

Cycle runWithoutOpponent(
  double ego_s,
  double ego_d,
  double heading_offset = 0.0,
  VehicleGeometry vehicle_geometry = VehicleGeometry{})
{
  return runOnce(
    ego_s, ego_d, heading_offset, 0.0, 0.0, false, defaultConfig(), vehicle_geometry);
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

TEST(RacingStateMachine, NearbyWallOutsideFiveCentimetreCorridorDoesNotTriggerOvertake)
{
  const RacelineReference reference = makeReference();
  const Odometry ego = egoAt(reference, 2.0, 0.0);
  OccupancyGrid grid = gridAround(ego.position, 8.0);
  stampOffsetWall(grid, reference, 3.0, 5.0, 0.15);

  RacingStateMachine machine(reference, defaultConfig(), VehicleGeometry{}, GridPolicy{});
  for (int i = 0; i < 5; ++i) {
    machine.update(ego, grid);
  }

  EXPECT_FALSE(machine.state().opponent.detected);
  EXPECT_EQ(machine.state().intent, PlannerIntent::FOLLOW_RACING_LINE);
}

// PRD 5: an offset car is never handed to the global follower.
TEST(RacingStateMachine, NoOpponentOffTheLineMerges)
{
  const Cycle cycle = runWithoutOpponent(2.0, 0.60);
  EXPECT_FALSE(cycle.state.opponent.detected);
  EXPECT_EQ(cycle.state.intent, PlannerIntent::MERGE);
}

// The gap is deliberately inside engagement_enter_gap_m rather than on it: at
// the boundary the detected face lands within an ULP of the threshold and the
// strict comparison in proposedIntent decides the test, not the behaviour.
TEST(RacingStateMachine, BehindInsideStartGapOvertakes)
{
  const Cycle cycle = runWithOpponentAtGap(2.0, 0.0, 1.5);
  ASSERT_TRUE(cycle.state.opponent.detected);
  EXPECT_EQ(cycle.state.relative_position, RelativePosition::BEHIND);
  EXPECT_NEAR(cycle.state.opponent.gap_m, 1.5, kFaceTolerance);
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
  VehicleGeometry vehicle_geometry;
  vehicle_geometry.collision_radius_m = 0.15;
  EXPECT_EQ(
    runWithoutOpponent(
      2.0, vehicle_geometry.fullWidthM() - 0.05, 0.0, vehicle_geometry).state.intent,
    PlannerIntent::FOLLOW_RACING_LINE);
  EXPECT_EQ(
    runWithoutOpponent(
      2.0, vehicle_geometry.fullWidthM() + 0.05, 0.0, vehicle_geometry).state.intent,
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

  RacingStateMachine machine(
    reference, defaultConfig(), VehicleGeometry{}, GridPolicy{});
  for (int i = 0; i < 5; ++i) {
    machine.update(ego, grid);
  }
  const TacticalState & state = machine.state();

  ASSERT_TRUE(state.opponent.detected);
  EXPECT_NEAR(state.opponent.gap_m, 0.50, kFaceTolerance);
  EXPECT_EQ(state.relative_position, RelativePosition::OVERLAPPING);
  // Ego is centred here, so the intent is OVERTAKE rather than PASS. The gap is
  // what this test is about; the intent follows from CentredEgoOvertakesInstead.
  EXPECT_EQ(state.intent, PlannerIntent::OVERTAKE);
}

// PASS holds an offset it cannot create: pass() and recover() both generate
// nothing inside a vehicle width, so a centred ego must not be handed PASS.
TEST(RacingStateMachine, CentredEgoOvertakesInsteadOfPassing)
{
  const Cycle centred = runWithOpponentAtGap(2.0, 0.10, 0.30);
  ASSERT_TRUE(centred.state.opponent.detected);
  EXPECT_EQ(centred.state.intent, PlannerIntent::OVERTAKE);

  // Same gap, displaced beyond a vehicle width: PASS is generatable and stands.
  const Cycle displaced = runWithOpponentAtGap(2.0, 0.50, 0.30);
  ASSERT_TRUE(displaced.state.opponent.detected);
  EXPECT_EQ(displaced.state.intent, PlannerIntent::PASS);
}

// Once ego overlaps or is ahead there is no overtake left to propose, so the
// gate stops applying and PASS stays the intent even from the racing line.
TEST(RacingStateMachine, CentredEgoStillPassesOnceOverlapping)
{
  const Cycle cycle = runWithOpponentAtGap(2.0, 0.10, -0.50);
  ASSERT_TRUE(cycle.state.opponent.detected);
  EXPECT_EQ(cycle.state.intent, PlannerIntent::PASS);
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

  RacingStateMachine machine(
    reference, defaultConfig(), VehicleGeometry{}, GridPolicy{});
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

  RacingStateMachine machine(
    reference, defaultConfig(), VehicleGeometry{}, GridPolicy{});
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

  RacingStateMachine machine(
    reference, defaultConfig(), VehicleGeometry{}, GridPolicy{});
  machine.update(Odometry{}, OccupancyGrid{});

  EXPECT_FALSE(machine.state().opponent.detected);
  EXPECT_EQ(machine.state().relative_position, RelativePosition::NONE);
  EXPECT_EQ(machine.state().intent, PlannerIntent::FOLLOW_RACING_LINE);
}

TEST(RacingStateMachine, OccupiedThresholdComesFromGridPolicy)
{
  const RacelineReference reference = makeReference();
  const Odometry ego = egoAt(reference, 2.0, 0.0);
  OccupancyGrid grid = gridAround(ego.position, 8.0);
  stampOpponent(grid, reference, 4.0, 5.0, 0.10, 60);

  GridPolicy strict_policy;
  strict_policy.occupied_threshold = 75;
  RacingStateMachine strict_machine(
    reference, defaultConfig(), VehicleGeometry{}, strict_policy);
  strict_machine.update(ego, grid);
  EXPECT_FALSE(strict_machine.state().opponent.detected);

  GridPolicy permissive_policy;
  permissive_policy.occupied_threshold = 50;
  RacingStateMachine permissive_machine(
    reference, defaultConfig(), VehicleGeometry{}, permissive_policy);
  permissive_machine.update(ego, grid);
  EXPECT_TRUE(permissive_machine.state().opponent.detected);
}

TEST(RacingStateMachine, UnknownCellsRemainFreeByDefault)
{
  const RacelineReference reference = makeReference();
  const Odometry ego = egoAt(reference, 2.0, 0.0);
  OccupancyGrid grid = gridAround(ego.position, 8.0);
  stampOpponent(grid, reference, 4.0, 5.0, 0.10, -1);

  RacingStateMachine machine(
    reference, defaultConfig(), VehicleGeometry{}, GridPolicy{});
  machine.update(ego, grid);

  EXPECT_FALSE(machine.state().opponent.detected);
}

TEST(RacingStateMachine, EmptyGridDetectsNothing)
{
  const RacelineReference reference = makeReference();
  const Odometry ego = egoAt(reference, 2.0, 0.0);

  RacingStateMachine machine(
    reference, defaultConfig(), VehicleGeometry{}, GridPolicy{});
  machine.update(ego, OccupancyGrid{});

  EXPECT_FALSE(machine.state().opponent.detected);
  EXPECT_EQ(machine.state().intent, PlannerIntent::FOLLOW_RACING_LINE);
}

TEST(RacingStateMachine, ReprocessingOneCostmapDoesNotAddOpponentEvidence)
{
  const RacelineReference reference = makeReference();
  const Odometry ego = egoAt(reference, 2.0, 0.0);
  OccupancyGrid grid = gridAround(ego.position, 8.0);
  stampOpponent(grid, reference, 3.5, 4.5);
  RacingStateMachine machine(reference, defaultConfig(), VehicleGeometry{}, GridPolicy{});

  machine.update(ego, grid, StateUpdateContext{0.0, 1});
  ASSERT_EQ(machine.state().proposed_intent, PlannerIntent::OVERTAKE);
  EXPECT_EQ(machine.state().pending_grid_count, 1U);
  machine.update(ego, grid, StateUpdateContext{0.10, 1});
  EXPECT_EQ(machine.state().intent, PlannerIntent::FOLLOW_RACING_LINE);
  EXPECT_EQ(machine.state().pending_grid_count, 1U);
  machine.update(ego, grid, StateUpdateContext{0.11, 2});
  EXPECT_EQ(machine.state().intent, PlannerIntent::OVERTAKE);
}

TEST(RacingStateMachine, FollowLateralBandAndMergeCompletionUseLocalizationCycles)
{
  const RacelineReference reference = makeReference();
  RacingStateMachine machine(reference, defaultConfig(), VehicleGeometry{}, GridPolicy{});
  OccupancyGrid grid = gridAround(reference.toCartesian(2.0, 0.0), 8.0);

  machine.update(egoAt(reference, 2.0, 0.20), grid, StateUpdateContext{0.0, 1});
  EXPECT_EQ(machine.state().intent, PlannerIntent::FOLLOW_RACING_LINE);
  machine.update(egoAt(reference, 2.0, 0.30), grid, StateUpdateContext{0.01, 1});
  machine.update(egoAt(reference, 2.0, 0.30), grid, StateUpdateContext{0.07, 1});
  ASSERT_EQ(machine.state().intent, PlannerIntent::MERGE);

  machine.update(egoAt(reference, 2.0, 0.20), grid, StateUpdateContext{0.08, 1});
  EXPECT_EQ(machine.state().intent, PlannerIntent::MERGE);
  machine.update(egoAt(reference, 2.0, 0.10), grid, StateUpdateContext{0.09, 1});
  machine.update(egoAt(reference, 2.0, 0.10), grid, StateUpdateContext{0.15, 1});
  EXPECT_EQ(machine.state().intent, PlannerIntent::FOLLOW_RACING_LINE);
}

TEST(RacingStateMachine, PassMergeTransitionsAreSlowAndMergeRequiresAProbe)
{
  const RacelineReference reference = makeReference();
  const Odometry ego = egoAt(reference, 2.0, 0.50);
  RacingStateMachine machine(reference, defaultConfig(), VehicleGeometry{}, GridPolicy{});
  const auto gridAtGap = [&](double gap) {
      OccupancyGrid grid = gridAround(ego.position, 8.0);
      const double start = gap >= 0.0 ? 2.0 + gap : 2.0 + gap - 1.0;
      stampOpponent(grid, reference, start, start + 1.0);
      return grid;
    };

  OccupancyGrid pass_grid = gridAtGap(0.20);
  machine.update(ego, pass_grid, StateUpdateContext{0.00, 1});
  machine.update(ego, pass_grid, StateUpdateContext{0.16, 2});
  ASSERT_EQ(machine.state().intent, PlannerIntent::PASS);

  OccupancyGrid clear_grid = gridAtGap(-2.0);
  machine.update(ego, clear_grid, StateUpdateContext{0.20, 3});
  machine.reportMergeProbe(true);
  machine.update(ego, clear_grid, StateUpdateContext{0.25, 4});
  machine.reportMergeProbe(true);
  machine.update(ego, clear_grid, StateUpdateContext{0.30, 5});
  machine.reportMergeProbe(true);
  EXPECT_EQ(machine.state().intent, PlannerIntent::PASS);
  machine.update(ego, clear_grid, StateUpdateContext{0.36, 6});
  ASSERT_EQ(machine.state().intent, PlannerIntent::MERGE);

  machine.update(ego, pass_grid, StateUpdateContext{0.40, 7});
  machine.update(ego, pass_grid, StateUpdateContext{0.50, 8});
  EXPECT_EQ(machine.state().intent, PlannerIntent::MERGE);
  machine.update(ego, pass_grid, StateUpdateContext{0.56, 9});
  EXPECT_EQ(machine.state().intent, PlannerIntent::PASS);
}

TEST(RacingStateMachine, ClockRollbackClearsPendingEvidence)
{
  const RacelineReference reference = makeReference();
  const Odometry ego = egoAt(reference, 2.0, 0.0);
  OccupancyGrid grid = gridAround(ego.position, 8.0);
  stampOpponent(grid, reference, 3.5, 4.5);
  RacingStateMachine machine(reference, defaultConfig(), VehicleGeometry{}, GridPolicy{});

  machine.update(ego, grid, StateUpdateContext{10.0, 1});
  ASSERT_EQ(machine.state().pending_grid_count, 1U);
  machine.update(ego, grid, StateUpdateContext{1.0, 2});
  EXPECT_EQ(machine.state().intent, PlannerIntent::FOLLOW_RACING_LINE);
  EXPECT_EQ(machine.state().pending_grid_count, 1U);
  EXPECT_DOUBLE_EQ(machine.state().pending_duration_s, 0.0);
}

TEST(RacingStateMachine, CompatiblePoseReturnsToFollowWithoutExtraCostmapEvidence)
{
  const RacelineReference reference = makeReference();
  const Odometry ego = egoAt(reference, 2.0, 0.0);
  OccupancyGrid opponent_grid = gridAround(ego.position, 8.0);
  stampOpponent(opponent_grid, reference, 3.5, 4.5);
  const OccupancyGrid empty_grid = gridAround(ego.position, 8.0);
  RacingStateMachine machine(reference, defaultConfig(), VehicleGeometry{}, GridPolicy{});

  machine.update(ego, opponent_grid, StateUpdateContext{0.00, 1});
  machine.update(ego, opponent_grid, StateUpdateContext{0.06, 2});
  ASSERT_EQ(machine.state().intent, PlannerIntent::OVERTAKE);
  machine.update(ego, empty_grid, StateUpdateContext{0.10, 3});
  EXPECT_EQ(machine.state().intent, PlannerIntent::OVERTAKE);
  machine.update(ego, empty_grid, StateUpdateContext{0.16, 3});
  EXPECT_EQ(machine.state().intent, PlannerIntent::FOLLOW_RACING_LINE);
  EXPECT_EQ(machine.state().pending_grid_count, 0U);
}

TEST(RacingStateMachine, GapBandsRetainTheCommittedState)
{
  const RacelineReference reference = makeReference();
  StateMachineConfig config = defaultConfig();
  config.fast_confirmation_s = 0.0;
  config.slow_confirmation_s = 0.0;
  config.opponent_confirmation_grids = 1;
  config.pass_merge_confirmation_grids = 1;
  config.merge_pass_confirmation_grids = 1;
  config.merge_probe_confirmation_cycles = 1;
  RacingStateMachine machine(reference, config, VehicleGeometry{}, GridPolicy{});
  const auto updateAtGap = [&](double gap, uint64_t sequence, double ego_d = 0.0) {
      const Odometry ego = egoAt(reference, 2.0, ego_d);
      OccupancyGrid grid = gridAround(ego.position, 8.0);
      const double start = gap >= 0.0 ? 2.0 + gap : 2.0 + gap - 1.0;
      stampOpponent(grid, reference, start, start + 1.0);
      machine.update(ego, grid, StateUpdateContext{
          0.01 * static_cast<double>(sequence), sequence});
    };

  updateAtGap(1.5, 1);
  ASSERT_EQ(machine.state().intent, PlannerIntent::OVERTAKE);
  updateAtGap(0.80, 2);
  EXPECT_EQ(machine.state().intent, PlannerIntent::OVERTAKE);
  updateAtGap(0.20, 3, 0.50);
  ASSERT_EQ(machine.state().intent, PlannerIntent::PASS);
  updateAtGap(0.80, 4, 0.50);
  EXPECT_EQ(machine.state().intent, PlannerIntent::PASS);
  updateAtGap(-1.00, 5, 0.50);
  EXPECT_EQ(machine.state().intent, PlannerIntent::PASS);
  updateAtGap(-2.00, 6, 0.50);
  machine.reportMergeProbe(true);
  updateAtGap(-2.00, 7, 0.50);
  ASSERT_EQ(machine.state().intent, PlannerIntent::MERGE);
  updateAtGap(-1.00, 8, 0.50);
  EXPECT_EQ(machine.state().intent, PlannerIntent::MERGE);
}

TEST(RacingStateMachine, EngagementBandRetainsWhetherOpponentIsRelevant)
{
  const RacelineReference reference = makeReference();
  StateMachineConfig config = defaultConfig();
  config.fast_confirmation_s = 0.0;
  config.slow_confirmation_s = 0.0;
  config.opponent_confirmation_grids = 1;
  RacingStateMachine machine(reference, config, VehicleGeometry{}, GridPolicy{});
  const Odometry ego = egoAt(reference, 2.0, 0.0);
  const auto updateAtGap = [&](double gap, uint64_t sequence) {
      OccupancyGrid grid = gridAround(ego.position, 8.0);
      stampOpponent(grid, reference, 2.0 + gap, 3.0 + gap);
      machine.update(ego, grid, StateUpdateContext{
          0.01 * static_cast<double>(sequence), sequence});
    };

  // Beyond engagement_enter_gap_m: the opponent is visible but not yet relevant.
  updateAtGap(2.20, 1);
  EXPECT_EQ(machine.state().intent, PlannerIntent::FOLLOW_RACING_LINE);
  updateAtGap(1.50, 2);
  ASSERT_EQ(machine.state().intent, PlannerIntent::OVERTAKE);
  // Inside the hysteresis band: past the enter gap, short of the exit gap.
  updateAtGap(2.30, 3);
  EXPECT_EQ(machine.state().intent, PlannerIntent::OVERTAKE);
  // Past engagement_exit_gap_m: the engagement is released.
  updateAtGap(2.80, 4);
  EXPECT_EQ(machine.state().intent, PlannerIntent::FOLLOW_RACING_LINE);
}

} // namespace
} // namespace local_planning
