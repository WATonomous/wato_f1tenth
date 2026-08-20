#include "local_planning/planning/local_planner.hpp"
#include "local_planning/core/geometry.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace local_planning
{
namespace
{

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

BoundaryState egoAt(const RacelineReference & reference, double s, double d)
{
  const ReferenceGeometrySample sample = reference.sampleAtS(s);
  return {
    sample.x + d * sample.normal_x,
    sample.y + d * sample.normal_y,
    sample.heading,
    sample.curvature / (1.0 - d * sample.curvature),
    sample.velocity};
}

ManeuverConfig testConfig()
{
  ManeuverConfig config;
  // The fixture's pass transition runs the full window, so the window has to be
  // long enough to hold it; the default horizon has since dropped to 4.0.
  config.horizon_m = 6.0;
  config.overtake_s_offsets_from_opponent_rear_m = {0.0};
  config.overtake_heading_offsets_rad = {0.0};
  config.passing_d_magnitudes_m = {0.55, 0.75};
  config.merge_completion_distances_m = {2.0};
  config.pass_transition_distances_m = {6.0};
  return config;
}

GridPolicy productionGridPolicy()
{
  GridPolicy policy;
  policy.treat_unknown_as_free = true;
  policy.treat_out_of_grid_as_free = true;
  return policy;
}

OccupancyGrid makeGrid(
  int width, int height, double resolution, double origin_x, double origin_y, int8_t fill)
{
  OccupancyGrid grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  grid.origin = Point(origin_x, origin_y);
  grid.data.assign(static_cast<std::size_t>(width * height), fill);
  return grid;
}

OccupancyGrid coveringGrid(int8_t fill)
{
  return makeGrid(400, 400, 0.20, -40.0, -40.0, fill);
}

void stampRacelineObstacle(
  OccupancyGrid & grid, const RacelineReference & reference, double start_s, double end_s)
{
  for (double s = start_s; s <= end_s; s += 0.05) {
    const Point point = reference.toCartesian(s, 0.0);
    const int col = static_cast<int>(std::floor((point.x - grid.origin.x) / grid.resolution));
    const int row = static_cast<int>(std::floor((point.y - grid.origin.y) / grid.resolution));
    if (col >= 0 && col < grid.width && row >= 0 && row < grid.height) {
      grid.data[static_cast<std::size_t>(row * grid.width + col)] = 100;
    }
  }
}

std::vector<TrackWidth> uniformWidths(
  const RacelineReference & reference, double right,
  double left)
{
  return std::vector<TrackWidth>(reference.waypointCount(), TrackWidth{right, left});
}

LocalPlanResult planIntent(
  RacelineReference & reference,
  OccupancyGrid & grid,
  PlannerIntent intent,
  double ego_s,
  double ego_d,
  double opponent_s = 5.0,
  PlannerIntent proposed_intent = PlannerIntent::FOLLOW_RACING_LINE,
  bool held_path_usable = false)
{
  const VehicleGeometry vehicle;
  const FrenetConnectionGenerator generator;
  const ManeuverBuilder builder(reference, generator, testConfig(), vehicle);
  LocalPlanner planner(
    reference, builder, vehicle, productionGridPolicy(), CollisionConfig{},
    VelocityProfileConfig{}, BrakingConfig{});
  planner.buildGridCache(grid);

  TacticalState state;
  state.intent = intent;
  state.proposed_intent = proposed_intent == PlannerIntent::FOLLOW_RACING_LINE ?
    intent : proposed_intent;
  state.ego_s = ego_s;
  state.ego_d = ego_d;
  state.opponent.detected = intent == PlannerIntent::OVERTAKE;
  state.opponent.s = opponent_s;
  state.opponent.gap_m = opponent_s - ego_s;
  return planner.plan(state, egoAt(reference, ego_s, ego_d), grid, held_path_usable);
}

bool offsetTailAt(const ManeuverCandidate & candidate, double magnitude)
{
  return candidate.uses_offset_tail &&
         std::abs(std::abs(candidate.passing_d) - magnitude) <= 1e-9;
}

std::size_t nonBrakingCandidateCount(const LocalPlanResult & result)
{
  return static_cast<std::size_t>(std::count_if(
    result.evaluated.begin(), result.evaluated.end(), [](const auto & item) {
             return item.source != CandidateSource::BRAKING;
    }));
}

}  // namespace

TEST(LocalPlanner, UnknownCellsThatViolateWidthAreRejected)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 0.40, 0.40), 0.10));

  OccupancyGrid grid = coveringGrid(-1);
  const LocalPlanResult result = planIntent(
    reference, grid, PlannerIntent::OVERTAKE, 2.0, 0.0);

  ASSERT_GT(result.decision.generated_count, 0u);
  EXPECT_EQ(result.decision.collision_rejected, 0u);
  EXPECT_EQ(result.decision.track_bounds_rejected, nonBrakingCandidateCount(result));
  EXPECT_EQ(result.decision.valid_candidate_count, 0u);
  // No maneuver survives, so the ladder falls through to braking rather than
  // leaving nothing selected.
  EXPECT_EQ(result.decision.executed_mode, ExecutedMode::BRAKING_FALLBACK);
}

TEST(LocalPlanner, KnownFreeCellsSkipWidthEvenWhenTheTableIsNarrow)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 0.40, 0.40), 0.10));

  OccupancyGrid grid = coveringGrid(0);
  const LocalPlanResult result = planIntent(
    reference, grid, PlannerIntent::OVERTAKE, 2.0, 0.0);

  ASSERT_GT(result.decision.generated_count, 0u);
  EXPECT_EQ(result.decision.track_bounds_rejected, 0u);
  EXPECT_GT(result.decision.valid_candidate_count, 0u);
  EXPECT_GE(result.selected_index, 0);
}

TEST(LocalPlanner, OccupiedCellsDieInCollisionBeforeWidth)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 0.40, 0.40), 0.10));

  OccupancyGrid grid = coveringGrid(100);
  const LocalPlanResult result = planIntent(
    reference, grid, PlannerIntent::OVERTAKE, 2.0, 0.0);

  ASSERT_GT(result.decision.generated_count, 0u);
  EXPECT_EQ(result.decision.collision_rejected, nonBrakingCandidateCount(result));
  EXPECT_EQ(result.decision.track_bounds_rejected, 0u);
  EXPECT_EQ(result.decision.valid_candidate_count, 0u);
}

TEST(LocalPlanner, UnknownPinchThenOpenKeepsInsideLineAndRejectsWideThroughPinch)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  std::vector<TrackWidth> widths = uniformWidths(reference, 2.0, 2.0);
  const double ds =
    reference.totalLength() / static_cast<double>(reference.waypointCount());
  for (std::size_t i = 0; i < widths.size(); ++i) {
    const double s = ds * static_cast<double>(i);
    if (s >= 2.2 && s <= 4.8) {
      widths[i] = {0.70, 0.70};
    }
  }
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      widths, 0.10));

  OccupancyGrid grid = coveringGrid(-1);
  const LocalPlanResult result = planIntent(
    reference, grid, PlannerIntent::OVERTAKE, 2.0, 0.0, 5.0);

  ASSERT_GT(result.decision.generated_count, 0u);
  EXPECT_GT(result.decision.track_bounds_rejected, 0u);
  EXPECT_GT(result.decision.valid_candidate_count, 0u);

  bool inside_tail_ok = false;
  for (const auto & evaluated : result.evaluated) {
    const auto & candidate =
      result.pool.at(static_cast<std::size_t>(evaluated.candidate_index));
    if (offsetTailAt(candidate, 0.75)) {
      EXPECT_FALSE(evaluated.track_bounds_ok);
      EXPECT_FALSE(evaluated.velocity_feasible);
    }
    if (offsetTailAt(candidate, 0.55)) {
      EXPECT_TRUE(evaluated.track_bounds_ok);
      EXPECT_TRUE(evaluated.velocity_feasible);
      inside_tail_ok = true;
    }
  }
  EXPECT_TRUE(inside_tail_ok);
}

TEST(LocalPlanner, OutOfGridSamplesUseWidthWhenTheCostmapCannotSee)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 0.40, 0.40), 0.10));

  const ReferenceGeometrySample ego = reference.sampleAtS(2.0);
  OccupancyGrid grid = makeGrid(16, 16, 0.10, ego.x - 0.80, ego.y - 0.80, 0);
  const LocalPlanResult result = planIntent(
    reference, grid, PlannerIntent::OVERTAKE, 2.0, 0.0);

  ASSERT_GT(result.decision.generated_count, 0u);
  EXPECT_EQ(result.decision.collision_rejected, 0u);
  EXPECT_EQ(result.decision.track_bounds_rejected, nonBrakingCandidateCount(result));
  EXPECT_EQ(result.decision.valid_candidate_count, 0u);
}

TEST(LocalPlanner, MergeThroughUnknownNeedsClearanceOnBothSides)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 2.0, 0.0), 0.10));

  OccupancyGrid unknown = coveringGrid(-1);
  const LocalPlanResult blocked = planIntent(
    reference, unknown, PlannerIntent::MERGE, 2.0, 0.10);
  ASSERT_GT(blocked.decision.generated_count, 0u);
  EXPECT_EQ(blocked.decision.valid_candidate_count, 0u);
  EXPECT_GT(blocked.decision.track_bounds_rejected, 0u);

  OccupancyGrid known_free = coveringGrid(0);
  const LocalPlanResult clear = planIntent(
    reference, known_free, PlannerIntent::MERGE, 2.0, 0.10);
  EXPECT_EQ(clear.decision.track_bounds_rejected, 0u);
  EXPECT_GT(clear.decision.valid_candidate_count, 0u);
}

TEST(LocalPlanner, DecisionPublishesRawBoundsAtEgo)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 1.0, 1.2), 0.10));

  OccupancyGrid grid = coveringGrid(0);
  const LocalPlanResult result = planIntent(
    reference, grid, PlannerIntent::OVERTAKE, 2.0, 0.0);
  const SustainableBounds bounds = reference.rawBounds(2.0);
  EXPECT_NEAR(result.decision.sustainable_right_m, bounds.right_magnitude, 1e-12);
  EXPECT_NEAR(result.decision.sustainable_left_m, bounds.left_magnitude, 1e-12);
}

TEST(LocalPlanner, PassKeepsExecutingPassWhileMergeProbeRuns)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ASSERT_TRUE(reference.setTrackWidths(uniformWidths(reference, 2.0, 2.0), 0.10));
  OccupancyGrid grid = coveringGrid(0);

  const LocalPlanResult result = planIntent(
    reference, grid, PlannerIntent::PASS, 2.0, 0.55, 5.0, PlannerIntent::MERGE);

  ASSERT_GE(result.merge_probe_index, 0);
  ASSERT_GE(result.selected_index, 0);
  EXPECT_TRUE(result.decision.merge_probe_available);
  EXPECT_EQ(result.decision.requested_intent, PlannerIntent::PASS);
  EXPECT_EQ(result.decision.executed_intent, PlannerIntent::PASS);
  const auto selected = std::find_if(
    result.evaluated.begin(), result.evaluated.end(), [&result](const auto & item) {
      return item.candidate_index == result.selected_index;
    });
  ASSERT_NE(selected, result.evaluated.end());
  EXPECT_NE(selected->source, CandidateSource::MERGE_PROBE);
}

TEST(LocalPlanner, NoSafeLocalGeometryReportsExplicitUnavailableRecovery)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ASSERT_TRUE(reference.setTrackWidths(uniformWidths(reference, 2.0, 2.0), 0.10));
  OccupancyGrid grid = coveringGrid(100);

  const LocalPlanResult result = planIntent(
    reference, grid, PlannerIntent::MERGE, 2.0, 0.55);

  EXPECT_EQ(result.decision.executed_mode, ExecutedMode::BRAKING_UNAVAILABLE);
  EXPECT_EQ(result.decision.recovery_reason, RecoveryReason::NO_SAFE_LOCAL_PATH);
  EXPECT_EQ(result.decision.candidate_source, CandidateSource::BRAKING);
  // Nowhere free to go still publishes geometry: full-length so the controller
  // keeps a lookahead point, and zero speed so it stops on a coherent steering
  // angle instead of the empty path that made it zero the wheel mid-corner.
  ASSERT_GE(result.selected_index, 0);
  const auto & path = result.pool.at(static_cast<std::size_t>(result.selected_index)).path;
  ASSERT_FALSE(path.empty());
  EXPECT_GE(path.back().s, 4.0);
  for (const auto & sample : path) {
    EXPECT_DOUBLE_EQ(sample.speed, 0.0);
  }
}

TEST(LocalPlanner, BrakingFallbackTurnsTowardTheRacelineWhenNoManeuverSurvives)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  // Track narrower than every offset the builder passes at, so no maneuver
  // survives the width check -- the off-line, nothing-selectable state braking
  // exists for.  The car sits inside the corridor, just off the line.
  ASSERT_TRUE(reference.setTrackWidths(uniformWidths(reference, 0.40, 0.40), 0.10));
  OccupancyGrid grid = coveringGrid(-1);

  const LocalPlanResult result = planIntent(
    reference, grid, PlannerIntent::OVERTAKE, 2.0, 0.35);

  ASSERT_EQ(result.decision.executed_mode, ExecutedMode::BRAKING_FALLBACK);
  EXPECT_EQ(result.decision.recovery_reason, RecoveryReason::BRAKING_FALLBACK);
  EXPECT_EQ(result.decision.candidate_source, CandidateSource::BRAKING);
  const auto & path = result.pool.at(static_cast<std::size_t>(result.selected_index)).path;
  ASSERT_FALSE(path.empty());
  EXPECT_GE(path.back().s, 4.0);
  // Speeds are braking's own, not the nominal profile's.
  EXPECT_LT(path.back().speed, path.front().speed);
  EXPECT_LE(std::abs(path.back().d), std::abs(path.front().d));
}

TEST(LocalPlanner, UsableHeldPathSkipsBrakingWhenNoManeuverSurvives)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ASSERT_TRUE(reference.setTrackWidths(uniformWidths(reference, 0.40, 0.40), 0.10));
  OccupancyGrid grid = coveringGrid(-1);

  const LocalPlanResult result = planIntent(
    reference, grid, PlannerIntent::OVERTAKE, 2.0, 0.35, 5.0,
    PlannerIntent::FOLLOW_RACING_LINE, true);

  EXPECT_EQ(result.decision.executed_mode, ExecutedMode::HELD_PATH);
  EXPECT_EQ(result.selected_index, -1);
  EXPECT_EQ(result.decision.candidate_source, CandidateSource::NONE);
  EXPECT_TRUE(std::none_of(result.evaluated.begin(), result.evaluated.end(), [](const auto & item) {
      return item.source == CandidateSource::BRAKING;
  }));
}

TEST(HeldPathPolicy, ReportsEveryActualHeldPublication)
{
  EXPECT_EQ(
    choosePublishedPath(
      ExecutedMode::HELD_PATH, false, true, true, 0.12, 0.10, 0.15),
    PublishedPathChoice::HELD);
  EXPECT_EQ(
    choosePublishedPath(
      ExecutedMode::MANEUVER, true, true, true, 0.05, 0.10, 0.15),
    PublishedPathChoice::HELD);
  EXPECT_EQ(
    choosePublishedPath(
      ExecutedMode::MANEUVER, true, true, true, 0.11, 0.10, 0.15),
    PublishedPathChoice::SELECTED);
}

TEST(HeldPathPolicy, RejectsInvalidExpiredAndIntentClearedHolds)
{
  EXPECT_EQ(
    choosePublishedPath(
      ExecutedMode::HELD_PATH, false, true, false, 0.05, 0.10, 0.15),
    PublishedPathChoice::NONE);
  EXPECT_EQ(
    choosePublishedPath(
      ExecutedMode::HELD_PATH, false, true, true, 0.15, 0.10, 0.15),
    PublishedPathChoice::NONE);
  EXPECT_EQ(
    choosePublishedPath(
      ExecutedMode::HELD_PATH, false, false, true, 0.05, 0.10, 0.15),
    PublishedPathChoice::NONE);
  EXPECT_EQ(
    choosePublishedPath(
      ExecutedMode::BRAKING_FALLBACK, true, true, true, 0.05, 0.10, 0.15),
    PublishedPathChoice::SELECTED);
}

TEST(LocalPlanner, FailedMergeFallsBackToGlobalRacelineWithoutTryingPass)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ASSERT_TRUE(reference.setTrackWidths(uniformWidths(reference, 2.0, 2.0), 0.10));
  OccupancyGrid grid = coveringGrid(0);
  stampRacelineObstacle(grid, reference, 2.8, 8.0);

  const LocalPlanResult result = planIntent(
    reference, grid, PlannerIntent::MERGE, 2.0, 0.75);

  EXPECT_EQ(result.selected_index, -1);
  EXPECT_EQ(result.decision.requested_intent, PlannerIntent::MERGE);
  EXPECT_EQ(result.decision.executed_intent, PlannerIntent::FOLLOW_RACING_LINE);
  EXPECT_EQ(result.decision.executed_mode, ExecutedMode::NO_LOCAL_PATH);
  EXPECT_EQ(result.decision.recovery_reason, RecoveryReason::MERGE_PATH_UNAVAILABLE);
  EXPECT_TRUE(std::none_of(result.evaluated.begin(), result.evaluated.end(), [](const auto & item) {
      return item.source == CandidateSource::PASS_PREFERRED ||
             item.source == CandidateSource::PASS_RECOVERY;
  }));
}

TEST(LocalPlanner, TrackRejectedMergeFallsBackToGlobalRaceline)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ASSERT_TRUE(reference.setTrackWidths(uniformWidths(reference, 0.40, 0.40), 0.10));
  OccupancyGrid grid = coveringGrid(-1);

  const LocalPlanResult result = planIntent(
    reference, grid, PlannerIntent::MERGE, 2.0, 0.75);

  EXPECT_EQ(result.selected_index, -1);
  EXPECT_EQ(result.decision.requested_intent, PlannerIntent::MERGE);
  EXPECT_EQ(result.decision.executed_intent, PlannerIntent::FOLLOW_RACING_LINE);
  EXPECT_EQ(result.decision.executed_mode, ExecutedMode::NO_LOCAL_PATH);
  EXPECT_EQ(result.decision.recovery_reason, RecoveryReason::MERGE_PATH_UNAVAILABLE);
  EXPECT_TRUE(std::none_of(result.evaluated.begin(), result.evaluated.end(), [](const auto & item) {
      return item.source == CandidateSource::BRAKING;
  }));
}

}  // namespace local_planning
