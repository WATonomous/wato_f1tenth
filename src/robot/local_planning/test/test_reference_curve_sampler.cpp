#include <gtest/gtest.h>

#include "local_planning/curves/reference_curve_sampler.hpp"

#include <cmath>
#include <limits>
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

}  // namespace

TEST(ReferenceCurveSampler, SamplesExactConstantOffsetGeometry)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const auto generated = ReferenceCurveSampler().generate(
    reference, {2.0, 2.35, 0.55, 0.1});

  ASSERT_TRUE(generated.valid);
  ASSERT_GT(generated.samples.size(), 2u);
  EXPECT_DOUBLE_EQ(generated.samples.front().s, 0.0);
  EXPECT_NEAR(reference.deltaS(2.0, generated.samples.back().raceline_s), 2.35, 1e-10);
  for (std::size_t i = 0; i < generated.samples.size(); ++i) {
    const CurveSample & sample = generated.samples[i];
    const ReferenceGeometrySample geometry = reference.sampleAtS(sample.raceline_s);
    const Point expected = reference.toCartesian(sample.raceline_s, 0.55);
    EXPECT_NEAR(sample.x, expected.x, 1e-10);
    EXPECT_NEAR(sample.y, expected.y, 1e-10);
    EXPECT_NEAR(sample.heading, geometry.heading, 1e-10);
    EXPECT_NEAR(
      sample.curvature,
      geometry.curvature / (1.0 - 0.55 * geometry.curvature), 1e-10);
    EXPECT_DOUBLE_EQ(sample.speed, 0.0);
    if (i > 0) {
      EXPECT_GT(sample.s, generated.samples[i - 1].s);
      EXPECT_LE(
        reference.deltaS(generated.samples[i - 1].raceline_s, sample.raceline_s),
        0.1 + 1e-10);
    }
  }
}

TEST(ReferenceCurveSampler, WrapsAcrossTheLapAndIncludesTheExactEnd)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const double start_s = reference.totalLength() - 0.25;
  const auto generated = ReferenceCurveSampler().generate(
    reference, {start_s, 0.5, -0.55, 0.1});

  ASSERT_TRUE(generated.valid);
  ASSERT_FALSE(generated.samples.empty());
  EXPECT_NEAR(generated.samples.front().raceline_s, start_s, 1e-10);
  EXPECT_NEAR(generated.samples.back().raceline_s, 0.25, 1e-10);
  const Point expected = reference.toCartesian(0.25, -0.55);
  EXPECT_NEAR(generated.samples.back().x, expected.x, 1e-10);
  EXPECT_NEAR(generated.samples.back().y, expected.y, 1e-10);
}

TEST(ReferenceCurveSampler, ZeroDistanceReturnsItsBoundary)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const auto generated = ReferenceCurveSampler().generate(
    reference, {3.0, 0.0, 0.0, 0.1});

  ASSERT_TRUE(generated.valid);
  ASSERT_EQ(generated.samples.size(), 1u);
  EXPECT_NEAR(generated.samples.front().raceline_s, 3.0, 1e-10);
}

TEST(ReferenceCurveSampler, RejectsInvalidSamplingAndSingularOffsets)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const ReferenceCurveSampler sampler;

  EXPECT_FALSE(sampler.generate(reference, {0.0, -1.0, 0.0, 0.1}).valid);
  EXPECT_FALSE(sampler.generate(reference, {0.0, 1.0, 0.0, 0.0}).valid);
  EXPECT_FALSE(sampler.generate(
      reference,
      {std::numeric_limits<double>::infinity(), 1.0, 0.0, 0.1}).valid);
  EXPECT_FALSE(sampler.generate(reference, {0.0, 1.0, 100.0, 0.1}).valid);
}

}  // namespace local_planning
