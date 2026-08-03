#include <gtest/gtest.h>

#include "local_planning/maneuvers/maneuver_builder.hpp"

#include <algorithm>
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

ManeuverBuilder makeBuilder(const RacelineReference & reference, ManeuverConfig config)
{
  static const CurveConnectionGenerator generator;
  return ManeuverBuilder(reference, generator, std::move(config));
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

} // namespace

TEST(ManeuverBuilder, OvertakeMirrorsOffsetsAndIsStrictlyForward)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ManeuverConfig config;
  config.overtake_s_offsets_from_opponent_rear_m = {0.0};
  config.passing_d_magnitudes_m = {0.55};
  config.overtake_heading_offsets_rad = {0.0};
  config.overtake_curvature_multipliers = {1.0};
  const ManeuverBuilder builder = makeBuilder(reference, config);
  const BoundaryState ego = egoAt(reference, 2.0, 0.0);

  const std::vector<ManeuverCandidate> both_sides = builder.overtake(ego, 2.0, 0.0, 5.0);
  ASSERT_EQ(both_sides.size(), 2u);
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
  config.overtake_curvature_multipliers = {0.0, 0.5, 1.0};
  const ManeuverBuilder builder = makeBuilder(reference, config);

  const std::vector<ManeuverCandidate> candidates =
    builder.overtake(egoAt(reference, 2.0, 0.0), 2.0, 0.0, 5.0);
  ASSERT_EQ(candidates.size(), 6u);
  const ReferenceGeometrySample reference_sample = reference.sampleAtS(5.0);
  for (int side_index = 0; side_index < 2; ++side_index) {
    const double d = side_index == 0 ? -0.55 : 0.55;
    const double offset_curvature =
      reference_sample.curvature / (1.0 - d * reference_sample.curvature);
    const std::vector<double> expected_curvatures{0.0, 0.5 * offset_curvature, offset_curvature};
    for (std::size_t i = 0; i < 3; ++i) {
      const Path & path = candidates[static_cast<std::size_t>(side_index) * 3u + i].path;
      const CurveSample * const intermediate = sampleAt(path, reference.toCartesian(5.0, d));
      ASSERT_NE(intermediate, nullptr);
      EXPECT_NEAR(intermediate->curvature, expected_curvatures[i], 1e-8);
      EXPECT_NEAR(endOffset(reference, path, 8.0), d, 1e-8);
    }
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

  ManeuverConfig invalid_curvature_multiplier;
  invalid_curvature_multiplier.overtake_curvature_multipliers = {1.1};
  EXPECT_THROW(makeBuilder(reference, invalid_curvature_multiplier), std::invalid_argument);
}

TEST(ManeuverBuilder, OvertakeAndMergePreserveForwardTargetsAcrossWrapAround)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ManeuverConfig config;
  config.overtake_s_offsets_from_opponent_rear_m = {0.0};
  config.passing_d_magnitudes_m = {0.55};
  config.overtake_heading_offsets_rad = {0.0};
  config.overtake_curvature_multipliers = {1.0};
  config.merge_completion_distances_m = {2.0};
  const ManeuverBuilder builder = makeBuilder(reference, config);

  const double ego_s = reference.totalLength() - 2.0;
  const std::vector<ManeuverCandidate> overtake = builder.overtake(
    egoAt(reference, ego_s, 0.0), ego_s, 0.0, 1.0);
  ASSERT_EQ(overtake.size(), 2u);
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

} // namespace local_planning
