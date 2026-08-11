#include <gtest/gtest.h>

#include "local_planning/maneuvers/maneuver_builder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

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

ManeuverBuilder makeBuilder(
  const RacelineReference & reference,
  ManeuverConfig config,
  VehicleGeometry vehicle_geometry = VehicleGeometry{})
{
  static const CurveConnectionGenerator generator;
  return ManeuverBuilder(
    reference, generator, std::move(config), vehicle_geometry);
}

const CurveSample * sampleAt(const Path & path, const Point & expected)
{
  const auto found = std::find_if(
    path.begin(), path.end(), [&](const CurveSample & sample) {
      return std::hypot(sample.x - expected.x, sample.y - expected.y) < 1e-6;
    });
  return found == path.end() ? nullptr : &*found;
}

double endOffset(const RacelineReference & reference, const Path & path, double seed_s)
{
  const CurveSample & end = path.back();
  return reference.project(Point(end.x, end.y), seed_s).d;
}

bool staysOnSide(
  const RacelineReference & reference,
  const Path & path,
  double ego_s,
  int side)
{
  double seed = ego_s;
  return std::all_of(path.begin(), path.end(),
           [&](const CurveSample & sample) {
             const Projection projection = reference.project(Point(sample.x, sample.y), seed);
             seed = projection.s;
             return side * projection.d > 1e-6;
    });
}

bool neverCrossesOppositeSide(
  const RacelineReference & reference,
  const Path & path,
  double ego_s,
  int side)
{
  double seed = ego_s;
  return std::all_of(path.begin(), path.end(),
           [&](const CurveSample & sample) {
             const Projection projection = reference.project(Point(sample.x, sample.y), seed);
             seed = projection.s;
             return side * projection.d >= -1e-6;
    });
}

} // namespace

TEST(ManeuverBuilder, OvertakeMirrorsOffsetsAndIsStrictlyForward)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ManeuverConfig config;
  config.overtake_s_offsets_from_opponent_rear_m = {0.0};
  config.passing_d_magnitudes_m = {0.55};
  config.overtake_heading_offsets_rad = {0.0};
  const ManeuverBuilder builder = makeBuilder(reference, config);
  const BoundaryState ego = egoAt(reference, 2.0, 0.0);

  const std::vector<ManeuverCandidate> both_sides = builder.overtake(ego, 2.0, 0.0, 5.0);
  ASSERT_EQ(both_sides.size(), 4u);
  EXPECT_LT(endOffset(reference, both_sides.front().path, 8.0), 0.0);
  EXPECT_GT(endOffset(reference, both_sides.back().path, 8.0), 0.0);
  EXPECT_TRUE(builder.overtake(ego, 2.0, 0.0, 8.0).empty());
}

TEST(ManeuverBuilder, OvertakeConnectsViaAnOffsetIntermediateTarget)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ManeuverConfig config;
  config.overtake_s_offsets_from_opponent_rear_m = {0.0};
  config.passing_d_magnitudes_m = {0.55};
  config.overtake_heading_offsets_rad = {0.15};
  const ManeuverBuilder builder = makeBuilder(reference, config);

  const std::vector<ManeuverCandidate> candidates =
    builder.overtake(egoAt(reference, 2.0, 0.0), 2.0, 0.0, 5.0);
  ASSERT_EQ(candidates.size(), 4u);
  const ReferenceGeometrySample reference_sample = reference.sampleAtS(5.0);
  for (int side_index = 0; side_index < 2; ++side_index) {
    const double d = side_index == 0 ? -0.55 : 0.55;
    const double offset_curvature =
      reference_sample.curvature / (1.0 - d * reference_sample.curvature);
    const std::array<double, 2> expected_curvatures{
      reference_sample.curvature, offset_curvature};
    for (std::size_t i = 0; i < expected_curvatures.size(); ++i) {
      const Path & path = candidates[
        static_cast<std::size_t>(side_index) * expected_curvatures.size() + i].path;
      EXPECT_TRUE(std::all_of(path.begin(), path.end(), [](const CurveSample & sample) {
          return std::isfinite(sample.raceline_s);
      }));
      const CurveSample * const intermediate = sampleAt(path, reference.toCartesian(5.0, d));
      ASSERT_NE(intermediate, nullptr);
      EXPECT_NEAR(reference.deltaS(5.0, intermediate->raceline_s), 0.0, 1e-8);
      EXPECT_NEAR(intermediate->curvature, expected_curvatures[i], 1e-8);
      EXPECT_NEAR(endOffset(reference, path, 8.0), d, 1e-8);
    }
  }
}

TEST(ManeuverBuilder, OvertakeCurvatureModesDoNotReproduceTightCornerCrossing)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(3.0, 240)));
  ManeuverConfig config;
  VehicleGeometry vehicle_geometry;
  config.horizon_m = 6.0;
  vehicle_geometry.collision_radius_m = 0.14;
  config.overtake_s_offsets_from_opponent_rear_m = {0.0};
  config.passing_d_magnitudes_m = {0.30};
  config.overtake_heading_offsets_rad = {0.0};
  const ManeuverBuilder builder = makeBuilder(reference, config, vehicle_geometry);

  const double ego_s = 2.0;
  const std::vector<ManeuverCandidate> candidates = builder.overtake(
    egoAt(reference, ego_s, 0.0), ego_s, 0.0, ego_s + 1.5);

  ASSERT_FALSE(candidates.empty());
  for (const ManeuverCandidate & candidate : candidates) {
    const int side = candidate.target_d > 0.0 ? 1 : -1;
    EXPECT_TRUE(neverCrossesOppositeSide(reference, candidate.path, ego_s, side));
  }
}

TEST(ManeuverBuilder, PassRecomputesTheNearestOffsetAndLeavesCenterlineRoutingToTheCaller)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const ManeuverBuilder builder = makeBuilder(reference, ManeuverConfig{});

  const std::vector<ManeuverCandidate> first =
    builder.pass(egoAt(reference, 4.0, 0.65), 4.0, 0.65);
  ASSERT_EQ(first.size(), 1u);
  EXPECT_NEAR(endOffset(reference, first.front().path, 10.0), 0.55, 1e-8);
  EXPECT_DOUBLE_EQ(first.front().target_d, 0.55);
  EXPECT_DOUBLE_EQ(first.front().maneuver_distance_m, 6.0);

  const std::vector<ManeuverCandidate> second =
    builder.pass(egoAt(reference, 4.2, 0.74), 4.2, 0.74);
  ASSERT_EQ(second.size(), 1u);
  EXPECT_NEAR(endOffset(reference, second.front().path, 10.2), 0.75, 1e-8);

  EXPECT_TRUE(builder.pass(egoAt(reference, 4.2, 0.0), 4.2, 0.0).empty());
  EXPECT_FALSE(builder.merge(egoAt(reference, 4.2, 0.0), 4.2).empty());
}

TEST(ManeuverBuilder, RecoveryUsesExactConstantOffsetTailsAndKeepsItsSide)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ManeuverConfig config;
  config.passing_d_magnitudes_m = {0.45, 0.60};
  config.pass_transition_distances_m = {2.0, 3.0, 2.0};
  const ManeuverBuilder builder = makeBuilder(reference, config);

  const std::vector<ManeuverCandidate> candidates =
    builder.recover(egoAt(reference, 6.0, 0.45), 6.0, 0.45);
  ASSERT_EQ(candidates.size(), 2u);
  const Path & first = candidates.front().path;
  EXPECT_NE(nullptr, sampleAt(first, reference.toCartesian(9.0, 0.60)));
  EXPECT_NE(nullptr, sampleAt(first, reference.toCartesian(9.1, 0.60)));
  EXPECT_NEAR(endOffset(reference, first, 12.0), 0.60, 1e-8);
  EXPECT_DOUBLE_EQ(candidates[0].maneuver_distance_m, 3.0);
  EXPECT_DOUBLE_EQ(candidates[1].maneuver_distance_m, 2.0);
  EXPECT_DOUBLE_EQ(candidates[0].target_d, 0.60);
  EXPECT_GT(candidates[0].max_offset_deviation_m, 0.0);
  for (const ManeuverCandidate & candidate : candidates) {
    EXPECT_TRUE(staysOnSide(reference, candidate.path, 6.0, 1));
  }
}

TEST(ManeuverBuilder, MergeUsesEveryUniqueCompletionDistanceAndAnExactRacelineTail)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ManeuverConfig config;
  config.merge_completion_distances_m = {1.0, 2.0, 6.0, 2.0};
  const ManeuverBuilder builder = makeBuilder(reference, config);

  const std::vector<ManeuverCandidate> candidates =
    builder.merge(egoAt(reference, 10.0, 0.10), 10.0);
  ASSERT_EQ(candidates.size(), 3u);
  EXPECT_NE(nullptr, sampleAt(candidates[1].path, reference.toCartesian(12.0, 0.0)));
  EXPECT_NE(nullptr, sampleAt(candidates[1].path, reference.toCartesian(12.1, 0.0)));
  EXPECT_NE(nullptr, sampleAt(candidates.back().path, reference.toCartesian(16.0, 0.0)));
  EXPECT_DOUBLE_EQ(candidates[0].maneuver_distance_m, 1.0);
  EXPECT_DOUBLE_EQ(candidates[1].maneuver_distance_m, 2.0);
  EXPECT_DOUBLE_EQ(candidates[2].maneuver_distance_m, 6.0);
}

TEST(ManeuverBuilder, RejectsConfigurationThatCannotRespectTheCommonHorizon)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));

  ManeuverConfig invalid_transition;
  invalid_transition.pass_transition_distances_m = {7.0};
  EXPECT_THROW(makeBuilder(reference, invalid_transition), std::invalid_argument);

  ManeuverConfig invalid_merge;
  invalid_merge.merge_completion_distances_m = {0.0};
  EXPECT_THROW(makeBuilder(reference, invalid_merge), std::invalid_argument);

  ManeuverConfig invalid_offsets;
  invalid_offsets.passing_d_magnitudes_m = {};
  EXPECT_THROW(makeBuilder(reference, invalid_offsets), std::invalid_argument);

  ManeuverConfig offsets_inside_deadband;
  offsets_inside_deadband.passing_d_magnitudes_m = {0.30, 0.55};
  EXPECT_THROW(makeBuilder(reference, offsets_inside_deadband), std::invalid_argument);
}

TEST(ManeuverBuilder, OvertakeAndMergePreserveForwardTargetsAcrossWrapAround)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ManeuverConfig config;
  config.overtake_s_offsets_from_opponent_rear_m = {0.0};
  config.passing_d_magnitudes_m = {0.55};
  config.overtake_heading_offsets_rad = {0.0};
  config.merge_completion_distances_m = {2.0};
  const ManeuverBuilder builder = makeBuilder(reference, config);

  const double ego_s = reference.totalLength() - 2.0;
  const std::vector<ManeuverCandidate> overtake = builder.overtake(
    egoAt(reference, ego_s, 0.0), ego_s, 0.0, 1.0);
  ASSERT_EQ(overtake.size(), 4u);
  EXPECT_NEAR(endOffset(reference, overtake.front().path, 4.0), -0.55, 1e-8);
  EXPECT_NEAR(endOffset(reference, overtake.back().path, 4.0), 0.55, 1e-8);

  const std::vector<ManeuverCandidate> merge = builder.merge(
    egoAt(reference, ego_s, 0.20), ego_s);
  ASSERT_EQ(merge.size(), 1u);
  EXPECT_NE(nullptr, sampleAt(merge.front().path, reference.toCartesian(0.0, 0.0)));
  EXPECT_NEAR(endOffset(reference, merge.front().path, 4.0), 0.0, 1e-8);
}

TEST(ManeuverBuilder, DenseSideValidationRejectsAnInconsistentMeasuredSide)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const ManeuverBuilder builder = makeBuilder(reference, ManeuverConfig{});

  EXPECT_TRUE(builder.pass(egoAt(reference, 4.0, -0.30), 4.0, 0.30).empty());
}

TEST(ManeuverBuilder, TrackBoundsFilterOvertakeSidesAndPassMovesInward)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ManeuverConfig config;
  config.overtake_s_offsets_from_opponent_rear_m = {0.0};
  config.overtake_heading_offsets_rad = {0.0};
  const ManeuverBuilder builder = makeBuilder(reference, config);

  uint32_t rejected = 0;
  const auto overtake = builder.overtake(
    egoAt(reference, 2.0, 0.0), 2.0, 0.0, 5.0,
    SustainableBounds{0.60, 0.40}, &rejected);
  ASSERT_EQ(overtake.size(), 2u);
  EXPECT_LT(overtake.front().target_d, 0.0);
  EXPECT_EQ(rejected, 3u);  // +0.55, -0.75, +0.75 configurations

  rejected = 0;
  const auto pass = builder.pass(
    egoAt(reference, 4.0, 0.74), 4.0, 0.74,
    SustainableBounds{1.0, 0.60}, &rejected);
  ASSERT_EQ(pass.size(), 1u);
  EXPECT_DOUBLE_EQ(pass.front().target_d, 0.55);
  EXPECT_EQ(rejected, 1u);
}

TEST(ManeuverBuilder, MergeRequiresClearanceOnBothSides)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const ManeuverBuilder builder = makeBuilder(reference, ManeuverConfig{});
  uint32_t rejected = 0;
  EXPECT_TRUE(builder.merge(
    egoAt(reference, 2.0, 0.2), 2.0, SustainableBounds{0.5, 0.0}, &rejected).empty());
  EXPECT_EQ(rejected, 1u);
}

} // namespace local_planning
