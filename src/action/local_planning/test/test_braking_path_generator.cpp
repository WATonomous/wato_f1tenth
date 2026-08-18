#include "local_planning/planning/braking_path_generator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadiusM = 10.0;

RacelineReference circleReference()
{
  constexpr int kCount = 200;
  std::vector<Point> points;
  points.reserve(kCount);
  for (int i = 0; i < kCount; ++i) {
    const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kCount);
    points.emplace_back(kRadiusM * std::cos(theta), kRadiusM * std::sin(theta), 5.0);
  }
  RacelineReference reference;
  EXPECT_TRUE(reference.setRacingLine(points));
  return reference;
}

RacelineReference unevenTightReference()
{
  constexpr int kCount = 80;
  std::vector<Point> points;
  points.reserve(kCount);
  for (int i = 0; i < kCount; ++i) {
    const double fraction = static_cast<double>(i) / static_cast<double>(kCount);
    const double theta = 2.0 * kPi * std::pow(fraction, 1.35);
    points.emplace_back(6.0 * std::cos(theta), 3.0 * std::sin(theta), 5.0);
  }
  RacelineReference reference;
  EXPECT_TRUE(reference.setRacingLine(points));
  return reference;
}

BrakingConfig defaultConfig()
{
  BrakingConfig config;
  config.horizon_m = 4.0;
  config.sample_spacing_m = 0.1;
  config.decel_mps2 = 5.0;
  config.min_velocity_mps = 1.0;
  config.friction_coeff = 1.0;
  config.max_steering_angle_rad = 0.52;
  config.wheelbase_m = 0.33;
  config.pursuit_lookaheads_m = {1.5};
  config.effort_levels = {1.0, 0.5, 0.0};
  return config;
}

// A car displaced outward from the line and yawed away from it -- the state a
// pure pursuit that ran wide through a corner actually leaves behind, and the
// one every Frenet connection back to d = 0 fails on.
BoundaryState wideAndYawed(
  const RacelineReference & reference, double s, double d, double heading_error, double speed)
{
  const auto geometry = reference.sampleAtS(s);
  BoundaryState ego;
  ego.x = geometry.x + d * geometry.normal_x;
  ego.y = geometry.y + d * geometry.normal_y;
  ego.heading = geometry.heading + heading_error;
  ego.speed = speed;
  return ego;
}

TEST(BrakingPathGenerator, GeneratesOneArcPerEffortLevelOnTheLine)
{
  const auto reference = circleReference();
  const auto config = defaultConfig();
  const BrakingPathGenerator generator(reference, config);

  // ego_d = 0 is exactly where pass() and recover() bail on sideOf() and leave
  // the pool empty; braking must still produce something here.
  const auto candidates = generator.generate(
    wideAndYawed(reference, 5.0, 0.0, 0.0, 4.0), 5.0, 0.0);
  ASSERT_EQ(candidates.size(), config.effort_levels.size());
  for (const auto & candidate : candidates) {
    ASSERT_FALSE(candidate.path.empty());
    EXPECT_GE(candidate.path.back().s, config.horizon_m - 1e-9);
  }
}

TEST(BrakingPathGenerator, ArcRunsTheFullHorizonAfterSpeedReachesTheFloor)
{
  const auto reference = circleReference();
  auto config = defaultConfig();
  config.effort_levels = {1.0};
  const BrakingPathGenerator generator(reference, config);

  // 1.2 m/s at 5 m/s^2 hits the floor within centimetres; the path must not
  // stop there, or find_lookahead() runs off its end and the car dead-stops.
  const auto candidates = generator.generate(
    wideAndYawed(reference, 5.0, 0.3, 0.2, 1.2), 5.0, 0.3);
  ASSERT_EQ(candidates.size(), 1u);
  EXPECT_GE(candidates.front().path.back().s, config.horizon_m - 1e-9);
}

TEST(BrakingPathGenerator, SpeedsDecayMonotonicallyToTheFloor)
{
  const auto reference = circleReference();
  const auto config = defaultConfig();
  const BrakingPathGenerator generator(reference, config);

  const auto candidates = generator.generate(
    wideAndYawed(reference, 5.0, 0.4, 0.3, 6.0), 5.0, 0.4);
  ASSERT_FALSE(candidates.empty());
  const auto & path = candidates.front().path;
  EXPECT_NEAR(path.front().speed, 6.0, 1e-9);
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_LE(path[i].speed, path[i - 1].speed + 1e-9);
    EXPECT_GE(path[i].speed, config.min_velocity_mps - 1e-9);
  }
  EXPECT_NEAR(path.back().speed, config.min_velocity_mps, 1e-9);
}

TEST(BrakingPathGenerator, CurvatureNeverExceedsTheBudgetAtThatSampleSpeed)
{
  const auto reference = circleReference();
  const auto config = defaultConfig();
  const BrakingPathGenerator generator(reference, config);

  const auto candidates = generator.generate(
    wideAndYawed(reference, 5.0, 0.6, 0.4, 7.0), 5.0, 0.6);
  ASSERT_FALSE(candidates.empty());
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const double effort = config.effort_levels[i];
    for (const auto & sample : candidates[i].path) {
      EXPECT_LE(
        std::abs(sample.curvature),
        effort * config.allowedCurvature(sample.speed) + 1e-9);
    }
  }
}

TEST(BrakingPathGenerator, FrictionBindsAboveTheCrossoverSpeedAndSteeringBelow)
{
  const auto config = defaultConfig();
  const double steering_limit = std::tan(0.52) / 0.33;
  // mu*g/v^2 = steering_limit at v = sqrt(9.81 / 1.74) ~ 2.37 m/s.
  EXPECT_NEAR(config.allowedCurvature(1.0), steering_limit, 1e-9);
  EXPECT_LT(config.allowedCurvature(6.0), steering_limit);
  EXPECT_NEAR(config.allowedCurvature(6.0), 9.81 / 36.0, 1e-9);
}

TEST(BrakingPathGenerator, EffortOrdersHowFastTheLineIsClosed)
{
  const auto reference = circleReference();
  const auto config = defaultConfig();
  const BrakingPathGenerator generator(reference, config);

  const auto candidates = generator.generate(
    wideAndYawed(reference, 5.0, 0.7, 0.25, 4.0), 5.0, 0.7);
  ASSERT_EQ(candidates.size(), 3u);
  const double start_d = candidates.front().path.front().d;
  EXPECT_NEAR(start_d, 0.7, 0.05);

  // Hard cut closes |d| the most, a straight brake the least, and the middle
  // sits between them.  This is the whole content of the effort parameter.
  constexpr std::size_t comparison_index = 10;
  ASSERT_GT(candidates[0].path.size(), comparison_index);
  EXPECT_LT(std::abs(candidates[0].path[comparison_index].d), std::abs(start_d));
  EXPECT_LT(
    std::abs(candidates[0].path[comparison_index].d),
    std::abs(candidates[1].path[comparison_index].d));
  EXPECT_LT(
    std::abs(candidates[1].path[comparison_index].d),
    std::abs(candidates[2].path[comparison_index].d));
}

TEST(BrakingPathGenerator, ShorterLookaheadClosesTheLineSooner)
{
  const auto reference = circleReference();
  auto config = defaultConfig();
  config.effort_levels = {1.0};
  config.pursuit_lookaheads_m = {1.0, 3.0};
  const BrakingPathGenerator generator(reference, config);
  const auto candidates = generator.generate(
    wideAndYawed(reference, 5.0, 0.7, 0.25, 4.0), 5.0, 0.7);
  ASSERT_EQ(candidates.size(), 2u);
  constexpr std::size_t comparison_index = 10;
  ASSERT_GT(candidates[0].path.size(), comparison_index);
  EXPECT_LT(
    std::abs(candidates[0].path[comparison_index].d),
    std::abs(candidates[1].path[comparison_index].d));
}

TEST(BrakingPathGenerator, ZeroEffortHoldsTheCurrentHeading)
{
  const auto reference = circleReference();
  auto config = defaultConfig();
  config.effort_levels = {0.0};
  const BrakingPathGenerator generator(reference, config);

  const auto ego = wideAndYawed(reference, 5.0, 0.7, 0.25, 4.0);
  const auto candidates = generator.generate(ego, 5.0, 0.7);
  ASSERT_EQ(candidates.size(), 1u);
  for (const auto & sample : candidates.front().path) {
    EXPECT_NEAR(sample.curvature, 0.0, 1e-12);
    EXPECT_NEAR(sample.heading, ego.heading, 1e-12);
  }
}

TEST(BrakingPathGenerator, EmptyWithoutAReference)
{
  RacelineReference reference;
  const BrakingPathGenerator generator(reference, defaultConfig());
  EXPECT_TRUE(generator.generate(BoundaryState{}, 0.0, 0.0).empty());
}

TEST(BrakingPathGenerator, FansEffortMajorAcrossLookaheads)
{
  const auto reference = circleReference();
  auto config = defaultConfig();
  config.effort_levels = {1.2, 0.0};
  config.pursuit_lookaheads_m = {1.0, 2.0, 3.0};
  const BrakingPathGenerator generator(reference, config);

  const auto params = generator.arcParams();
  const auto candidates = generator.generate(
    wideAndYawed(reference, 5.0, 0.5, 0.2, 4.0), 5.0, 0.5);
  ASSERT_EQ(params.size(), 6u);
  ASSERT_EQ(candidates.size(), params.size());
  EXPECT_DOUBLE_EQ(params[0].effort, 1.0);
  EXPECT_DOUBLE_EQ(params[0].lookahead_m, 1.0);
  EXPECT_DOUBLE_EQ(params[2].lookahead_m, 3.0);
  EXPECT_DOUBLE_EQ(params[3].effort, 0.0);
  EXPECT_DOUBLE_EQ(params[3].lookahead_m, 1.0);
}

TEST(BrakingPathGenerator, FirstSampleKeepsTheProvidedProjection)
{
  const auto reference = circleReference();
  const BrakingPathGenerator generator(reference, defaultConfig());
  constexpr double ego_s = 5.0;
  constexpr double ego_d = 0.7;
  const auto ego = wideAndYawed(reference, ego_s, ego_d, 0.25, 4.0);

  const auto candidates = generator.generate(ego, ego_s, ego_d);
  ASSERT_FALSE(candidates.empty());
  const auto & first = candidates.front().path.front();
  EXPECT_DOUBLE_EQ(first.raceline_s, ego_s);
  EXPECT_DOUBLE_EQ(first.d, ego_d);
  EXPECT_DOUBLE_EQ(first.x, ego.x);
  EXPECT_DOUBLE_EQ(first.y, ego.y);
}

TEST(BrakingPathGenerator, CorrectedStationsMatchSeededProjection)
{
  const auto reference = circleReference();
  const BrakingPathGenerator generator(reference, defaultConfig());
  const auto ego = wideAndYawed(reference, 5.0, 0.8, 0.35, 5.0);
  const auto candidates = generator.generate(ego, 5.0, 0.8);
  ASSERT_FALSE(candidates.empty());

  for (const auto & sample : candidates.front().path) {
    const auto ref = reference.sampleAtS(sample.raceline_s);
    const double px = sample.x - ref.x;
    const double py = sample.y - ref.y;
    EXPECT_NEAR(px * ref.tangent_x + py * ref.tangent_y, 0.0, 1e-4);
    const auto projected = reference.project(Point(sample.x, sample.y), sample.raceline_s);
    EXPECT_NEAR(reference.deltaS(sample.raceline_s, projected.s), 0.0, 1e-4);
    EXPECT_NEAR(sample.d, projected.d, 1e-4);
  }
}

TEST(BrakingPathGenerator, CorrectionAccountsForNonUnitSplineParameterSpeed)
{
  const auto reference = unevenTightReference();
  bool found_non_unit_speed = false;
  for (double s = 0.0; s < reference.totalLength(); s += 0.05) {
    found_non_unit_speed |= std::abs(reference.sampleAtS(s).parameter_speed - 1.0) > 1e-3;
  }
  ASSERT_TRUE(found_non_unit_speed);

  const BrakingPathGenerator generator(reference, defaultConfig());
  const auto ego = wideAndYawed(reference, 4.0, 0.6, 0.3, 4.0);
  const auto candidates = generator.generate(ego, 4.0, 0.6);
  ASSERT_FALSE(candidates.empty());
  for (const auto & sample : candidates.front().path) {
    const auto projected = reference.project(Point(sample.x, sample.y), sample.raceline_s);
    EXPECT_NEAR(reference.deltaS(sample.raceline_s, projected.s), 0.0, 1e-4);
    EXPECT_NEAR(sample.d, projected.d, 1e-4);
  }
}

TEST(BrakingPathGenerator, CorrectedStationWrapsAcrossTheLoopSeam)
{
  const auto reference = circleReference();
  auto config = defaultConfig();
  config.horizon_m = 1.0;
  const double ego_s = reference.totalLength() - 0.15;
  const BrakingPathGenerator generator(reference, config);
  const auto ego = wideAndYawed(reference, ego_s, 0.1, 0.0, 3.0);
  const auto candidates = generator.generate(ego, ego_s, 0.1);
  ASSERT_FALSE(candidates.empty());

  bool crossed_seam = false;
  const auto & path = candidates.front().path;
  for (std::size_t i = 1; i < path.size(); ++i) {
    crossed_seam |= path[i].raceline_s < path[i - 1].raceline_s;
    const double advance = reference.deltaS(path[i - 1].raceline_s, path[i].raceline_s);
    EXPECT_GT(advance, 0.0);
    EXPECT_LT(advance, 0.3);
  }
  EXPECT_TRUE(crossed_seam);
}

TEST(BrakingPathGenerator, IntegratesEachStepAsAnExactCircularArc)
{
  const auto reference = circleReference();
  auto config = defaultConfig();
  config.effort_levels = {1.0};
  const BrakingPathGenerator generator(reference, config);
  const auto ego = wideAndYawed(reference, 5.0, 0.7, 0.25, 4.0);
  const auto candidates = generator.generate(ego, 5.0, 0.7);
  ASSERT_EQ(candidates.size(), 1u);
  ASSERT_GE(candidates.front().path.size(), 2u);

  const auto & first = candidates.front().path[0];
  const auto & second = candidates.front().path[1];
  const double half_turn = 0.5 * first.curvature * config.sample_spacing_m;
  const double chord = std::abs(half_turn) < 1e-12 ? config.sample_spacing_m :
    config.sample_spacing_m * std::sin(half_turn) / half_turn;
  EXPECT_NEAR(second.x, first.x + chord * std::cos(first.heading + half_turn), 1e-12);
  EXPECT_NEAR(second.y, first.y + chord * std::sin(first.heading + half_turn), 1e-12);
  EXPECT_NEAR(second.heading, first.heading + first.curvature * config.sample_spacing_m, 1e-12);
}

}  // namespace
}  // namespace local_planning
