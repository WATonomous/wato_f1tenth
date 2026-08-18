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
  static const FrenetConnectionGenerator generator;
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

// These used to project every sample back onto the reference to recover its
// offset.  The sample carries it, and carrying it is the point of the port, so
// asserting on sample.d is both cheaper and a stronger check: it verifies the
// value downstream stages actually consume rather than an independent estimate
// of it.
double endOffset(const Path & path)
{
  return path.back().d;
}

bool staysOnSide(const Path & path, int side)
{
  return std::all_of(path.begin(), path.end(),
           [side](const CurveSample & sample) {return side * sample.d > 1e-6;});
}

// The candidate's max_abs_d_m is accumulated leg by leg as the path is sampled,
// never by sweeping the finished path.  Sweeping it here is what makes the
// assertion worth writing: the two have to agree across a multi-leg candidate.
double sweptMaxAbsOffset(const Path & path)
{
  double worst = 0.0;
  for (const CurveSample & sample : path) {
    worst = std::max(worst, std::abs(sample.d));
  }
  return worst;
}

bool neverCrossesOppositeSide(const Path & path, int side)
{
  return std::all_of(path.begin(), path.end(),
           [side](const CurveSample & sample) {return side * sample.d >= -1e-6;});
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
  // Two two-leg candidates (one per side) and two tails.  Was six: the
  // (curvature mode) axis doubled the two-leg count with duplicates.
  ASSERT_EQ(both_sides.size(), 4u);
  EXPECT_EQ(std::count_if(
      both_sides.begin(), both_sides.end(),
      [](const ManeuverCandidate & candidate) {return candidate.uses_offset_tail;}), 2);
  EXPECT_LT(endOffset(both_sides.front().path), 0.0);
  EXPECT_GT(endOffset(both_sides.back().path), 0.0);
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
  // Four, not the old six: the (curvature mode) axis of the product is gone.
  // It existed as a solver hedge, and with d'' = 0 both of its values request
  // the same boundary, so it was producing exact duplicates.
  ASSERT_EQ(candidates.size(), 4u);
  const ReferenceGeometrySample reference_sample = reference.sampleAtS(5.0);
  for (double d : {-0.55, 0.55}) {
    // d'' = 0 at the intermediate boundary gives kappa_ref / (1 - d kappa_ref)
    // exactly -- what the old OFFSET mode asked the G2 solver for.
    const double offset_curvature =
      reference_sample.curvature / (1.0 - d * reference_sample.curvature);
    std::vector<const ManeuverCandidate *> two_leg;
    for (const ManeuverCandidate & candidate : candidates) {
      if (!candidate.uses_offset_tail && std::abs(candidate.passing_d - d) <= 1e-8) {
        two_leg.push_back(&candidate);
      }
    }
    ASSERT_EQ(two_leg.size(), 1u);
    const Path & path = two_leg.front()->path;
    EXPECT_TRUE(std::all_of(path.begin(), path.end(), [](const CurveSample & sample) {
        return std::isfinite(sample.raceline_s);
    }));
    const CurveSample * const intermediate = sampleAt(path, reference.toCartesian(5.0, d));
    ASSERT_NE(intermediate, nullptr);
    EXPECT_NEAR(reference.deltaS(5.0, intermediate->raceline_s), 0.0, 1e-8);
    EXPECT_NEAR(intermediate->d, d, 1e-12);
    // The heading offset means exactly what it says at the boundary.
    EXPECT_NEAR(
      std::atan2(
        std::sin(intermediate->heading - reference_sample.heading),
        std::cos(intermediate->heading - reference_sample.heading)),
      0.15, 1e-12);
    // Not offset_curvature.  d'' = 0 reproduces kappa_ref / (1 - d kappa_ref)
    // only where d' = 0; here the boundary is turned away from the tangent, and
    // the d' terms of the curvature formula are live.  The old code requested
    // the constant-offset curvature from the solver regardless of the heading
    // offset, which was asking for the curvature of a curve the path was not
    // tangent to.  It is now a few 1e-4 away, and consistently so.
    EXPECT_GT(std::abs(intermediate->curvature - offset_curvature), 1e-5);
    EXPECT_LT(std::abs(intermediate->curvature - offset_curvature), 1e-3);
    EXPECT_NEAR(endOffset(path), d, 1e-12);
  }
}

TEST(ManeuverBuilder, OvertakeOffsetTailsAreExactAndG2Continuous)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ManeuverConfig config;
  config.overtake_s_offsets_from_opponent_rear_m = {0.0};
  config.passing_d_magnitudes_m = {0.55};
  // The exact-tail entry is intentionally independent of this exploratory grid.
  config.overtake_heading_offsets_rad = {0.15};
  const ManeuverBuilder builder = makeBuilder(reference, config);

  const auto candidates = builder.overtake(
    egoAt(reference, 2.0, 0.0), 2.0, 0.0, 5.0);
  for (double d : {-0.55, 0.55}) {
    const auto found = std::find_if(
      candidates.begin(), candidates.end(), [d](const ManeuverCandidate & candidate) {
        return candidate.uses_offset_tail && std::abs(candidate.passing_d - d) <= 1e-8;
      });
    ASSERT_NE(found, candidates.end());

    const Point join_point = reference.toCartesian(5.0, d);
    const auto join = std::find_if(
      found->path.begin(), found->path.end(), [&](const CurveSample & sample) {
        return std::hypot(sample.x - join_point.x, sample.y - join_point.y) < 1e-6;
      });
    ASSERT_NE(join, found->path.end());
    const ReferenceGeometrySample join_reference = reference.sampleAtS(5.0);
    EXPECT_NEAR(
      std::atan2(
        std::sin(join->heading - join_reference.heading),
        std::cos(join->heading - join_reference.heading)),
      0.0, 1e-8);
    EXPECT_NEAR(
      join->curvature,
      join_reference.curvature / (1.0 - d * join_reference.curvature), 1e-8);

    for (auto sample = join; sample != found->path.end(); ++sample) {
      // Exact, not "converged to within 1e-7": the tail is a constant-d
      // connection, so d is the coefficient it was built from.
      EXPECT_NEAR(sample->d, d, 1e-12);
    }
  }
}

TEST(ManeuverBuilder, DefaultOvertakeAddsOneOffsetTailPerStationAndOffset)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const ManeuverBuilder builder = makeBuilder(reference, ManeuverConfig{});

  const auto candidates = builder.overtake(
    egoAt(reference, 2.0, 0.0), 2.0, 0.0, 4.0);
  EXPECT_EQ(std::count_if(
      candidates.begin(), candidates.end(),
      [](const ManeuverCandidate & candidate) {return candidate.uses_offset_tail;}), 8);
}

TEST(ManeuverBuilder, OvertakeNeverCrossesTheRacelineInATightCorner)
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
    const int side = candidate.terminal_d > 0.0 ? 1 : -1;
    EXPECT_TRUE(neverCrossesOppositeSide(candidate.path, side));
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
  EXPECT_NEAR(endOffset(first.front().path), 0.55, 1e-8);
  EXPECT_DOUBLE_EQ(first.front().terminal_d, 0.55);
  EXPECT_DOUBLE_EQ(first.front().maneuver_distance_m, ManeuverConfig{}.horizon_m);

  const std::vector<ManeuverCandidate> second =
    builder.pass(egoAt(reference, 4.2, 0.74), 4.2, 0.74);
  ASSERT_EQ(second.size(), 1u);
  EXPECT_NEAR(endOffset(second.front().path), 0.75, 1e-8);

  EXPECT_TRUE(builder.pass(egoAt(reference, 4.2, 0.0), 4.2, 0.0).empty());
  EXPECT_FALSE(builder.merge(egoAt(reference, 4.2, 0.0), 4.2, 0.0).empty());
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
  ASSERT_EQ(candidates.size(), 4u);
  const auto first = std::find_if(
    candidates.begin(), candidates.end(), [](const ManeuverCandidate & candidate) {
      return candidate.terminal_d == 0.60 && candidate.maneuver_distance_m == 3.0;
    });
  ASSERT_NE(first, candidates.end());
  EXPECT_NE(nullptr, sampleAt(first->path, reference.toCartesian(9.0, 0.60)));
  EXPECT_NE(nullptr, sampleAt(first->path, reference.toCartesian(9.1, 0.60)));
  EXPECT_NEAR(endOffset(first->path), 0.60, 1e-8);
  // Two legs -- the 3 m transition and the constant-offset tail -- so this also
  // checks the accumulator carries across the join.
  EXPECT_NEAR(first->max_abs_d_m, sweptMaxAbsOffset(first->path), 1e-12);
  EXPECT_GE(first->max_abs_d_m, 0.60);
  for (const ManeuverCandidate & candidate : candidates) {
    EXPECT_TRUE(candidate.uses_offset_tail);
    EXPECT_TRUE(staysOnSide(candidate.path, 1));
  }
}

TEST(ManeuverBuilder, RecoveryAddsShortPreferredTailsWithoutDuplicatingNominalPass)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ManeuverConfig config;
  config.horizon_m = 6.0;
  config.pass_transition_distances_m = {6.0, 3.0, 1.0};
  const ManeuverBuilder builder = makeBuilder(reference, config);

  const auto candidates = builder.recover(
    egoAt(reference, 6.0, 0.55), 6.0, 0.55);
  std::vector<double> preferred_tail_distances;
  for (const ManeuverCandidate & candidate : candidates) {
    if (candidate.uses_offset_tail && std::abs(candidate.passing_d - 0.55) <= 1e-8) {
      preferred_tail_distances.push_back(candidate.maneuver_distance_m);
    }
  }
  ASSERT_EQ(preferred_tail_distances.size(), 2u);
  EXPECT_DOUBLE_EQ(preferred_tail_distances[0], 3.0);
  EXPECT_DOUBLE_EQ(preferred_tail_distances[1], 1.0);
  EXPECT_TRUE(std::none_of(
      candidates.begin(), candidates.end(), [](const ManeuverCandidate & candidate) {
        return std::abs(candidate.terminal_d - 0.55) <= 1e-8 &&
               std::abs(candidate.maneuver_distance_m - 6.0) <= 1e-8;
      }));
}

TEST(ManeuverBuilder, MergeUsesEveryUniqueCompletionDistanceAndAnExactRacelineTail)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ManeuverConfig config;
  config.horizon_m = 6.0;
  config.merge_completion_distances_m = {1.0, 2.0, 6.0, 2.0};
  const ManeuverBuilder builder = makeBuilder(reference, config);

  const std::vector<ManeuverCandidate> candidates =
    builder.merge(egoAt(reference, 10.0, 0.10), 10.0, 0.10);
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
  EXPECT_NEAR(endOffset(overtake.front().path), -0.55, 1e-8);
  EXPECT_NEAR(endOffset(overtake.back().path), 0.55, 1e-8);
  const auto wrapped_tail = std::find_if(
    overtake.begin(), overtake.end(), [](const ManeuverCandidate & candidate) {
      return candidate.uses_offset_tail && candidate.passing_d < 0.0;
    });
  ASSERT_NE(wrapped_tail, overtake.end());
  EXPECT_NE(
    nullptr, sampleAt(wrapped_tail->path, reference.toCartesian(1.1, -0.55)));

  const std::vector<ManeuverCandidate> merge = builder.merge(
    egoAt(reference, ego_s, 0.20), ego_s, 0.20);
  ASSERT_EQ(merge.size(), 1u);
  EXPECT_NE(nullptr, sampleAt(merge.front().path, reference.toCartesian(0.0, 0.0)));
  EXPECT_NEAR(endOffset(merge.front().path), 0.0, 1e-8);
}

// The dense side sweep is what lets the selector rank PASS on (safety, time)
// without re-checking which side a candidate is on.  A connection out of a
// straight-ahead ego is monotone and could not cross anyway; the sweep earns
// its keep when the measured heading points across the line, which makes the
// quintic dip before it recovers.
TEST(ManeuverBuilder, PassCandidatesNeverCrossToTheFarSide)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  const ManeuverBuilder builder = makeBuilder(reference, ManeuverConfig{}, vehicle);

  const double ego_d = 0.55;
  const double deadband = vehicle.fullWidthM();
  bool saw_a_rejection = false;
  for (const double heading_error :
    {-1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0})
  {
    BoundaryState ego = egoAt(reference, 4.0, ego_d);
    ego.heading += heading_error;
    const auto candidates = builder.pass(ego, 4.0, ego_d);
    if (candidates.empty()) {
      saw_a_rejection = true;
      continue;
    }
    for (const ManeuverCandidate & candidate : candidates) {
      // The contract is a deadband, not zero: inside one vehicle width of the
      // line still counts as "our side".
      for (const CurveSample & sample : candidate.path) {
        EXPECT_GT(sample.d, -deadband) << "heading_error=" << heading_error;
      }
    }
  }
  // A heading turned hard across the line must not still produce a candidate at
  // every setting, or the sweep is not doing anything.
  EXPECT_TRUE(saw_a_rejection);
}

// A centred car has no side to pass on; routing that case is the caller's job.
TEST(ManeuverBuilder, PassDeclinesWhenTheCarIsOnTheLine)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const ManeuverBuilder builder = makeBuilder(reference, ManeuverConfig{});

  EXPECT_TRUE(builder.pass(egoAt(reference, 4.0, 0.10), 4.0, 0.10).empty());
  EXPECT_TRUE(builder.recover(egoAt(reference, 4.0, 0.10), 4.0, 0.10).empty());
}

TEST(ManeuverBuilder, OvertakeGeneratesAllConfiguredOffsetsAndPassPrefersNearest)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ManeuverConfig config;
  config.overtake_s_offsets_from_opponent_rear_m = {0.0};
  config.overtake_heading_offsets_rad = {0.0};
  const ManeuverBuilder builder = makeBuilder(reference, config);

  const auto overtake = builder.overtake(
    egoAt(reference, 2.0, 0.0), 2.0, 0.0, 5.0);
  // One station x four offsets x (two same-side horizon offsets) = eight
  // two-leg candidates, plus four tails.  Was twenty before the curvature axis
  // went away.
  ASSERT_EQ(overtake.size(), 12u);
  EXPECT_TRUE(std::any_of(
      overtake.begin(), overtake.end(), [](const ManeuverCandidate & candidate) {
        return candidate.terminal_d < 0.0;
      }));
  EXPECT_TRUE(std::any_of(
      overtake.begin(), overtake.end(), [](const ManeuverCandidate & candidate) {
        return candidate.terminal_d > 0.0;
      }));
  EXPECT_EQ(std::count_if(
      overtake.begin(), overtake.end(), [](const ManeuverCandidate & candidate) {
        return candidate.uses_offset_tail;
      }), 4);

  const auto pass = builder.pass(egoAt(reference, 4.0, 0.74), 4.0, 0.74);
  ASSERT_EQ(pass.size(), 1u);
  EXPECT_DOUBLE_EQ(pass.front().terminal_d, 0.75);
}

TEST(ManeuverBuilder, MergeGeneratesWithoutInjectedWidthCaps)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  ManeuverConfig config;
  config.merge_completion_distances_m = {2.0};
  const ManeuverBuilder builder = makeBuilder(reference, config);
  EXPECT_FALSE(builder.merge(egoAt(reference, 2.0, 0.2), 2.0, 0.2).empty());
}

} // namespace local_planning
