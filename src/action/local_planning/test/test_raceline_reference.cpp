#include "local_planning/reference/raceline_reference.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// A closed circle of radius R sampled at `count` points.  Analytic ground
// truth: curvature is 1/R everywhere and arc length is 2*pi*R.
std::vector<Point> circleLine(double radius, int count, double speed = 3.0)
{
  std::vector<Point> points;
  points.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(count);
    points.emplace_back(radius * std::cos(theta), radius * std::sin(theta), speed);
  }
  return points;
}

// Two straights 2*half_gap apart joined by 180-degree turns at each end: a
// closed loop whose long sides are anti-parallel and close together.  This is
// the hairpin geometry that breaks globally-nearest projection.
std::vector<Point> hairpinLine(double straight_length, double half_gap, double spacing)
{
  std::vector<Point> points;
  const int straight_steps = static_cast<int>(straight_length / spacing);
  const int arc_steps = static_cast<int>(kPi * half_gap / spacing);

  // Outbound along +x at y = +half_gap.
  for (int i = 0; i < straight_steps; ++i) {
    points.emplace_back(static_cast<double>(i) * spacing, half_gap, 3.0);
  }
  // Right-hand turn at the far end, from heading +x to heading -x.
  for (int i = 0; i < arc_steps; ++i) {
    const double theta = kPi * static_cast<double>(i) / static_cast<double>(arc_steps);
    points.emplace_back(
      straight_length + half_gap * std::sin(theta),
      half_gap * std::cos(theta),
      3.0);
  }
  // Return along -x at y = -half_gap.
  for (int i = 0; i < straight_steps; ++i) {
    points.emplace_back(straight_length - static_cast<double>(i) * spacing, -half_gap, 3.0);
  }
  // Turn at the near end, back to heading +x.
  for (int i = 0; i < arc_steps; ++i) {
    const double theta = kPi * static_cast<double>(i) / static_cast<double>(arc_steps);
    points.emplace_back(
      -half_gap * std::sin(theta),
      -half_gap * std::cos(theta),
      3.0);
  }
  return points;
}

} // namespace

// Roundtrip: converting (s, d) to Cartesian and projecting back must return
// what went in, across a dense sweep that crosses the wrap point and reaches
// the maximum offset the planner will ever sample.
TEST(RacelineReference, RoundtripAcrossWrapAndMaxOffset)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(5.0, 120)));

  const double length = reference.totalLength();
  EXPECT_NEAR(length, 2.0 * kPi * 5.0, 0.02);

  for (int i = 0; i < 400; ++i) {
    // Start just below zero so the sweep crosses the wrap point.
    const double s = -1.0 + length * static_cast<double>(i) / 400.0 * 1.05;
    for (const double d : {-1.8, -0.5, 0.0, 0.5, 1.8}) {
      const Point p = reference.toCartesian(s, d);
      const double heading = reference.sampleAtS(s).heading;
      const Projection back = reference.project(p, heading, reference.wrapS(s));

      EXPECT_NEAR(reference.deltaS(reference.wrapS(s), back.s), 0.0, 1e-3)
        << "s=" << s << " d=" << d;
      EXPECT_NEAR(back.d, d, 1e-3) << "s=" << s << " d=" << d;
      EXPECT_FALSE(back.seed_was_stale) << "s=" << s << " d=" << d;
    }
  }
}

// Curvature and speed come from the same representation as position, so a
// circle must report 1/R everywhere rather than the staircase a piecewise
// approximation produces.
TEST(RacelineReference, CurvatureMatchesAnalyticCircle)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(5.0, 120, 4.0)));

  for (int i = 0; i < 200; ++i) {
    const double s = reference.totalLength() * static_cast<double>(i) / 200.0;
    const ReferenceGeometrySample sample = reference.sampleAtS(s);
    EXPECT_NEAR(std::abs(sample.curvature), 1.0 / 5.0, 1e-3) << "s=" << s;
    EXPECT_NEAR(sample.velocity, 4.0, 1e-9);
    EXPECT_NEAR(std::hypot(sample.tangent_x, sample.tangent_y), 1.0, 1e-9);
  }
}

// The reason the tangent check exists, and the whole reason global-nearest is
// not shippable.  Ego is driving the outbound branch (heading +x) but has
// drifted across the centreline, so it is geometrically *nearer* the return
// branch.  Nearest-by-distance answers with the wrong branch and s jumps by
// most of a lap in one cycle; the wrong branch is anti-parallel, so comparing
// tangents is what separates them.
//
// Both halves are asserted deliberately: without the global-nearest half this
// test would pass on geometry where the correct branch is also the nearest one,
// which is to say it would not be testing anything.
TEST(RacelineReference, TangentCheckKeepsProjectionOnEgoBranch)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(hairpinLine(10.0, 0.6, 0.1)));

  const double ego_heading = 0.0;          // travelling +x, the outbound branch
  const Point ego(5.0, -0.10, 0.0);        // nearer the return branch at y = -0.6

  // Nearest by distance, orientation ignored: lands on the return branch.
  const Projection nearest = reference.projectGlobal(ego, ego_heading, false);
  ASSERT_LT(reference.sampleAtS(nearest.s).y, 0.0)
    << "test geometry no longer discriminates: the nearest branch is already "
       "the correct one, so this test would pass without the tangent check";

  // Seeded from where ego was a moment ago on the outbound branch, with the
  // tangent check: stays on the branch ego is actually driving.
  const Projection seeded = reference.project(ego, ego_heading, 4.9);
  const ReferenceGeometrySample landed = reference.sampleAtS(seeded.s);
  EXPECT_GT(landed.y, 0.0) << "seeded projection jumped to the return branch";
  EXPECT_FALSE(seeded.seed_was_stale);
  EXPECT_NEAR(landed.heading, ego_heading, 0.3);

  // And the jump the tangent check prevents is enormous, which is why its
  // absence is silent rather than obviously wrong.
  EXPECT_GT(std::abs(reference.deltaS(seeded.s, nearest.s)), 5.0);
}

// Continuity: replaying poses around the loop, s must advance smoothly and
// never jump. A violated bound here is the earliest symptom of branch
// confusion in the field.
TEST(RacelineReference, DeltaSStaysBoundedAroundLoop)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(hairpinLine(10.0, 0.6, 0.1)));

  const double length = reference.totalLength();
  double seed = 0.0;
  for (int i = 1; i <= 500; ++i) {
    const double true_s = length * static_cast<double>(i) / 500.0;
    const ReferenceGeometrySample sample = reference.sampleAtS(true_s);
    const Point pose(sample.x + 0.15 * sample.normal_x, sample.y + 0.15 * sample.normal_y);

    const Projection projection = reference.project(pose, sample.heading, seed);
    const double step = reference.deltaS(seed, projection.s);
    EXPECT_GT(step, -0.05) << "s went backwards at i=" << i;
    EXPECT_LT(step, length / 100.0) << "s jumped at i=" << i;
    EXPECT_FALSE(projection.seed_was_stale) << "at i=" << i;
    seed = projection.s;
  }
}

TEST(RacelineReference, RejectsDegenerateInput)
{
  RacelineReference reference;
  EXPECT_FALSE(reference.setRacingLine({}));
  EXPECT_FALSE(reference.setRacingLine({Point(0, 0), Point(1, 0)}));
  EXPECT_FALSE(reference.valid());
}

TEST(RacelineReference, TrackWidthsApplyClearanceAndForwardMinimumAcrossWrap)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  std::vector<TrackWidth> widths(reference.waypointCount());
  for (std::size_t i = 0; i < reference.waypointCount(); ++i) {
    widths[i] = {1.0, 1.2};
  }
  widths[0] = {0.40, 0.80};

  ASSERT_TRUE(reference.setTrackWidths(widths, 3.0, 0.20, 0.05, 0.10));
  const auto bounds = reference.sustainableBounds(reference.totalLength() - 0.05);
  EXPECT_NEAR(bounds.right_magnitude, 0.15, 1e-5);
  EXPECT_NEAR(bounds.left_magnitude, 0.55, 1e-5);
}

TEST(RacelineReference, TrackWidthsRequireOneWidthPerWaypoint)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const std::vector<TrackWidth> widths(reference.waypointCount() - 1, {1.0, 1.0});
  EXPECT_FALSE(reference.setTrackWidths(widths, 6.0, 0.20, 0.05, 0.10));
  EXPECT_FALSE(reference.trackWidthsValid());
}

} // namespace local_planning
