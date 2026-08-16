#include "local_planning/collision/track_bounds_checker.hpp"

#include <gtest/gtest.h>

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
    const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(count);
    points.emplace_back(radius * std::cos(theta), radius * std::sin(theta), 3.0);
  }
  return points;
}

OccupancyGrid makeGrid(
  int width, int height, double resolution, double origin_x, double origin_y,
  int8_t fill)
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

std::vector<TrackWidth> uniformWidths(
  const RacelineReference & reference, double right, double left)
{
  return std::vector<TrackWidth>(reference.waypointCount(), TrackWidth{right, left});
}

CurveSample offsetSample(const RacelineReference & reference, double s, double d)
{
  const auto geometry = reference.sampleAtS(s);
  CurveSample sample;
  sample.x = geometry.x + d * geometry.normal_x;
  sample.y = geometry.y + d * geometry.normal_y;
  sample.heading = geometry.heading;
  sample.raceline_s = s;
  return sample;
}

std::vector<CurveSample> offsetPath(
  const RacelineReference & reference, double s0, double s1, double d,
  double ds = 0.20)
{
  std::vector<CurveSample> path;
  for (double s = s0; s <= s1 + 1e-9; s += ds) {
    path.push_back(offsetSample(reference, s, d));
    path.back().s = s - s0;
  }
  return path;
}

void stampFootprint(
  OccupancyGrid & grid, const CurveSample & sample, const VehicleGeometry & vehicle,
  int8_t value)
{
  const Point centers[] = {
    Point(sample.x, sample.y),
    Point(
      sample.x + vehicle.front_circle_offset_m * std::cos(sample.heading),
      sample.y + vehicle.front_circle_offset_m * std::sin(sample.heading)),
  };
  for (const Point & p : centers) {
    const int col = static_cast<int>(
      std::floor((p.x - grid.origin.x) / grid.resolution));
    const int row = static_cast<int>(
      std::floor((p.y - grid.origin.y) / grid.resolution));
    if (col < 0 || col >= grid.width || row < 0 || row >= grid.height) {
      continue;
    }
    grid.data[static_cast<std::size_t>(row * grid.width + col)] = value;
  }
}

}  // namespace

TEST(TrackBoundsChecker, MissingWidthsAreOk)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const TrackBoundsChecker checker(reference, VehicleGeometry{});
  const auto result = checker.check(
    offsetPath(reference, 2.0, 4.0, 0.75), coveringGrid(-1));
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.station_hint_samples, 0u);
  EXPECT_EQ(result.station_hint_fallbacks, 0u);
}

TEST(TrackBoundsChecker, UnknownWideOffsetIsRejected)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 0.40, 0.40), 0.10));
  const TrackBoundsChecker checker(reference, vehicle);

  const auto result = checker.check(
    offsetPath(reference, 2.0, 4.0, 0.55), coveringGrid(-1));
  EXPECT_FALSE(result.ok);
  EXPECT_GT(result.station_hint_samples, 0u);
  EXPECT_EQ(result.station_hint_fallbacks, 0u);
}

TEST(TrackBoundsChecker, KnownFreeSkipsWidth)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 0.40, 0.40), 0.10));
  const TrackBoundsChecker checker(reference, vehicle);

  const auto result = checker.check(
    offsetPath(reference, 2.0, 4.0, 0.55), coveringGrid(0));
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.station_hint_samples, 0u);
}

TEST(TrackBoundsChecker, KnownOccupiedSkipsWidth)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 0.40, 0.40), 0.10));
  const TrackBoundsChecker checker(reference, vehicle);

  const auto result = checker.check(
    offsetPath(reference, 2.0, 4.0, 0.55), coveringGrid(100));
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.station_hint_samples, 0u);
}

TEST(TrackBoundsChecker, OutOfGridUsesWidth)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 0.40, 0.40), 0.10));
  const TrackBoundsChecker checker(reference, vehicle);

  const auto ego = reference.sampleAtS(2.0);
  OccupancyGrid grid = makeGrid(4, 4, 0.10, ego.x - 0.20, ego.y - 0.20, 0);
  const auto result = checker.check(offsetPath(reference, 2.0, 6.0, 0.55), grid);
  EXPECT_FALSE(result.ok);
  EXPECT_GT(result.station_hint_samples, 0u);
}

TEST(TrackBoundsChecker, UnknownOnRacelineStaysInsideBounds)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 0.0, 0.0), 0.10));
  const TrackBoundsChecker checker(reference, vehicle);

  const auto result = checker.check(
    offsetPath(reference, 2.0, 4.0, 0.0), coveringGrid(-1));
  EXPECT_TRUE(result.ok);
  EXPECT_GT(result.station_hint_samples, 0u);
  EXPECT_EQ(result.station_hint_fallbacks, 0u);
}

TEST(TrackBoundsChecker, KnownPrefixThenUnknownSuffixChecksTheUnseen)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 0.40, 0.40), 0.10));
  const TrackBoundsChecker checker(reference, vehicle);

  const auto known = offsetPath(reference, 2.0, 3.0, 0.55);
  const auto unseen = offsetPath(reference, 3.2, 4.2, 0.55);
  auto path = known;
  path.insert(path.end(), unseen.begin(), unseen.end());

  OccupancyGrid grid = coveringGrid(-1);
  for (const CurveSample & sample : known) {
    stampFootprint(grid, sample, vehicle, 0);
  }

  const auto result = checker.check(path, grid);
  EXPECT_FALSE(result.ok);
  EXPECT_GT(result.station_hint_samples, 0u);
}

TEST(TrackBoundsChecker, PreviousStationSeedsNewtonAfterBadHint)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 1.0, 1.0), 0.10));
  const TrackBoundsChecker checker(reference, vehicle);

  std::vector<CurveSample> path = {
    offsetSample(reference, 2.0, 0.0),
    offsetSample(reference, 2.2, 0.0),
  };
  path[1].raceline_s = 12.0;

  OccupancyGrid grid = coveringGrid(-1);
  stampFootprint(grid, path[0], vehicle, 0);

  const auto result = checker.check(path, grid);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.station_hint_samples, 2u);
  EXPECT_EQ(result.station_hint_fallbacks, 0u);
}

TEST(TrackBoundsChecker, ProjectFallbackWhenBothHintsMiss)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(30.0, 240)));
  const VehicleGeometry vehicle;
  ASSERT_TRUE(reference.setTrackWidths(
      uniformWidths(reference, 0.40, 0.40), 0.10));
  const TrackBoundsChecker checker(reference, vehicle);

  CurveSample sample = offsetSample(reference, 2.0, 0.55);
  sample.raceline_s = 12.0;

  const auto result = checker.check({sample}, coveringGrid(-1));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.station_hint_samples, 1u);
  EXPECT_EQ(result.station_hint_fallbacks, 1u);
}

}  // namespace local_planning
