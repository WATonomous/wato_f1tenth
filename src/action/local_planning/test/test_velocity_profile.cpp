#include "local_planning/speed/velocity_profile.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.81;

std::vector<Point> straightLoop(double length, double speed, double spacing = 0.5)
{
  // Thin closed rectangle so projection stays on the +x outbound branch.
  std::vector<Point> points;
  const int steps = std::max(2, static_cast<int>(std::ceil(length / spacing)));
  for (int i = 0; i < steps; ++i) {
    points.emplace_back(static_cast<double>(i) * spacing, 0.0, speed);
  }
  points.emplace_back(length, 0.2, speed);
  for (int i = steps; i >= 0; --i) {
    points.emplace_back(static_cast<double>(i) * spacing, 0.4, speed);
  }
  points.emplace_back(0.0, 0.2, speed);
  return points;
}

CurveSample sampleAt(
  double s,
  double x,
  double y,
  double heading = 0.0,
  double curvature = 0.0)
{
  CurveSample sample;
  sample.s = s;
  sample.x = x;
  sample.y = y;
  sample.heading = heading;
  sample.curvature = curvature;
  sample.speed = -1.0;  // sentinel: must not remain if profiling fails
  sample.raceline_s = x;
  return sample;
}

std::vector<CurveSample> straightPath(double length, double spacing, double curvature = 0.0)
{
  std::vector<CurveSample> path;
  const int steps = std::max(1, static_cast<int>(std::ceil(length / spacing)));
  for (int i = 0; i <= steps; ++i) {
    const double x = length * static_cast<double>(i) / static_cast<double>(steps);
    path.push_back(sampleAt(x, x, 0.0, 0.0, curvature));
  }
  return path;
}

VelocityProfileConfig defaultConfig()
{
  VelocityProfileConfig config;
  config.friction_coeff = 1.0;
  config.min_velocity_mps = 0.0;
  config.max_velocity_mps = 10.0;
  config.max_accel_mps2 = 5.0;
  config.max_decel_mps2 = 5.0;
  config.overtake_speed_scale = 1.1;
  return config;
}

} // namespace

TEST(VelocityProfile, StraightRacelineAndVehicleCaps)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(straightLoop(20.0, 4.0)));
  VelocityProfileConfig config = defaultConfig();
  config.max_velocity_mps = 3.0;

  std::vector<CurveSample> path = straightPath(5.0, 0.5);
  const VelocityProfileResult result = assignVelocityProfile(
    path, 3.0, 0.0, 5.0, PlannerIntent::FOLLOW_RACING_LINE, reference, config);

  ASSERT_TRUE(result.feasible);
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_LE(path[i].speed, config.max_velocity_mps + 1e-9);
    EXPECT_LE(path[i].speed, 4.0 + 1e-9);  // raceline speed
  }
  EXPECT_NEAR(path.front().speed, 3.0, 1e-9);
}

TEST(VelocityProfile, CurvatureFrictionCap)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(straightLoop(20.0, 10.0)));
  VelocityProfileConfig config = defaultConfig();
  config.max_velocity_mps = 20.0;

  const double curvature = 0.5;
  const double friction_limit = std::sqrt(config.friction_coeff * kGravity / curvature);
  std::vector<CurveSample> path = straightPath(4.0, 0.5, curvature);

  const VelocityProfileResult result = assignVelocityProfile(
    path, 0.0, 0.0, 4.0, PlannerIntent::FOLLOW_RACING_LINE, reference, config);

  ASSERT_TRUE(result.feasible);
  for (const CurveSample & sample : path) {
    EXPECT_LE(sample.speed, friction_limit + 1e-6);
  }
}

TEST(VelocityProfile, InteriorScalingByIntent)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(straightLoop(30.0, 5.0)));
  VelocityProfileConfig config = defaultConfig();
  config.max_velocity_mps = 20.0;
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;

  auto profile_mid = [&](PlannerIntent intent) {
      std::vector<CurveSample> path = straightPath(6.0, 0.5);
      const VelocityProfileResult result = assignVelocityProfile(
        path, 5.0, 0.0, 6.0, intent, reference, config);
      EXPECT_TRUE(result.feasible);
      return path[path.size() / 2].speed;
    };

  EXPECT_NEAR(profile_mid(PlannerIntent::FOLLOW_RACING_LINE), 5.0, 0.15);
  EXPECT_NEAR(profile_mid(PlannerIntent::OVERTAKE), 5.5, 0.15);
  EXPECT_NEAR(profile_mid(PlannerIntent::PASS), 5.5, 0.15);
  EXPECT_NEAR(profile_mid(PlannerIntent::MERGE), 5.5, 0.15);
}

TEST(VelocityProfile, ExplicitTerminalCapAllModes)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(straightLoop(30.0, 5.0)));
  VelocityProfileConfig config = defaultConfig();
  config.max_velocity_mps = 20.0;
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;

  auto terminal_speed = [&](PlannerIntent intent) {
      std::vector<CurveSample> path = straightPath(6.0, 0.5);
      const VelocityProfileResult result = assignVelocityProfile(
        path, 5.0, 0.0, 6.0, intent, reference, config);
      EXPECT_TRUE(result.feasible);
      return path.back().speed;
    };

  EXPECT_NEAR(terminal_speed(PlannerIntent::FOLLOW_RACING_LINE), 5.0, 0.05);
  EXPECT_NEAR(terminal_speed(PlannerIntent::OVERTAKE), 5.5, 0.05);
  EXPECT_NEAR(terminal_speed(PlannerIntent::PASS), 5.5, 0.05);
  // MERGE hands off at unscaled raceline speed.
  EXPECT_NEAR(terminal_speed(PlannerIntent::MERGE), 5.0, 0.05);
}

TEST(VelocityProfile, TerminalCapPropagatesBackward)
{
  RacelineReference reference;
  // Slow terminal station on an otherwise fast line.
  std::vector<Point> points = straightLoop(30.0, 8.0);
  // Overwrite speeds so stations near s=6 are slow.
  for (Point & point : points) {
    if (point.x >= 5.5 && point.y < 0.2) {
      point.velocity = 2.0;
    }
  }
  ASSERT_TRUE(reference.setRacingLine(points));

  VelocityProfileConfig config = defaultConfig();
  config.max_velocity_mps = 20.0;
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 2.0;

  std::vector<CurveSample> path = straightPath(6.0, 0.25);
  const VelocityProfileResult result = assignVelocityProfile(
    path, 2.0, 0.0, 6.0, PlannerIntent::FOLLOW_RACING_LINE, reference, config);

  ASSERT_TRUE(result.feasible);
  EXPECT_NEAR(path.back().speed, 2.0, 0.2);
  // Deceleration from earlier samples must already be below the unconstrained
  // raceline speed because of the terminal cap.
  const std::size_t mid = path.size() / 2;
  EXPECT_LT(path[mid].speed, 8.0 - 0.5);
}

// The profile no longer launches from the measured speed -- see the comment on
// the feasibility check in assignVelocityProfile.  What survives is that the
// forward pass still bounds how fast speeds may rise between samples.
TEST(VelocityProfile, StartIsNotAnEchoOfTheMeasuredSpeed)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(straightLoop(30.0, 10.0)));
  VelocityProfileConfig config = defaultConfig();
  config.max_velocity_mps = 10.0;
  config.max_accel_mps2 = 2.0;
  config.max_decel_mps2 = 100.0;

  std::vector<CurveSample> path = straightPath(4.0, 0.5);
  const VelocityProfileResult result = assignVelocityProfile(
    path, 0.0, 0.0, 4.0, PlannerIntent::FOLLOW_RACING_LINE, reference, config);

  ASSERT_TRUE(result.feasible);
  // Measured start was 0.0.  A command of 0.0 here would stall the car forever.
  EXPECT_GT(path.front().speed, 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double ds = std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    const double max_from_accel = std::sqrt(
      path[i - 1].speed * path[i - 1].speed + 2.0 * config.max_accel_mps2 * ds);
    EXPECT_LE(path[i].speed, max_from_accel + 1e-6);
  }
}

// A measured speed above what the path can accept is a request to decelerate,
// not grounds to throw the path away.  This is the rule the braking family was
// already exempt from, and denying it to every other family is what left the
// candidate pool empty at speed.
TEST(VelocityProfile, MeasuredStartAboveTheCeilingCommandsASlowdownInsteadOfRejecting)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(straightLoop(30.0, 2.0)));
  VelocityProfileConfig config = defaultConfig();
  config.max_velocity_mps = 20.0;
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 1.0;

  std::vector<CurveSample> path = straightPath(2.0, 0.5);
  for (CurveSample & sample : path) {
    sample.speed = -1.0;
  }

  // 10 m/s measured against a 2 m/s raceline with 2 m of path and 1 m/s^2 of
  // decel: the car cannot reach the ceiling within the horizon.  It is still a
  // usable path -- it is the only thing that tells the car to slow down.
  const VelocityProfileResult result = assignVelocityProfile(
    path, 10.0, 0.0, 2.0, PlannerIntent::FOLLOW_RACING_LINE, reference, config);

  ASSERT_TRUE(result.feasible);
  for (const CurveSample & sample : path) {
    EXPECT_GT(sample.speed, 0.0);
    EXPECT_LT(sample.speed, 10.0);
  }
  // The sample at the car commands the ceiling, which here is the raceline
  // speed, so the controller sees a slowdown of 8 m/s rather than nothing.
  EXPECT_NEAR(path.front().speed, 2.0, 1e-6);
}

TEST(VelocityProfile, RacelineWrapAroundSequentialProjection)
{
  RacelineReference reference;
  std::vector<Point> circle;
  const double radius = 5.0;
  const int count = 80;
  for (int i = 0; i < count; ++i) {
    const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(count);
    circle.emplace_back(radius * std::cos(theta), radius * std::sin(theta), 3.0);
  }
  ASSERT_TRUE(reference.setRacingLine(circle));

  const double length = reference.totalLength();
  const double start_s = length - 1.0;
  std::vector<CurveSample> path;
  for (int i = 0; i <= 20; ++i) {
    const double s = start_s + 0.1 * static_cast<double>(i);
    const ReferenceGeometrySample geo = reference.sampleAtS(s);
    CurveSample sample = sampleAt(
      0.1 * static_cast<double>(i), geo.x, geo.y, geo.heading, geo.curvature);
    sample.raceline_s = s;
    path.push_back(sample);
  }

  VelocityProfileConfig config = defaultConfig();
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;

  const VelocityProfileResult result = assignVelocityProfile(
    path, 3.0, start_s, start_s + 2.0, PlannerIntent::FOLLOW_RACING_LINE, reference, config);

  ASSERT_TRUE(result.feasible);
  for (const CurveSample & sample : path) {
    EXPECT_NEAR(sample.speed, 3.0, 0.2);
  }
}

TEST(VelocityProfile, TraversalTimeConstantSpeed)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(straightLoop(30.0, 4.0)));
  VelocityProfileConfig config = defaultConfig();
  config.max_velocity_mps = 4.0;
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;

  const double length = 4.0;
  std::vector<CurveSample> path = straightPath(length, 0.5);
  const VelocityProfileResult result = assignVelocityProfile(
    path, 4.0, 0.0, length, PlannerIntent::FOLLOW_RACING_LINE, reference, config);

  ASSERT_TRUE(result.feasible);
  EXPECT_NEAR(result.traversal_time_s, length / 4.0, 1e-6);
}

TEST(VelocityProfile, UsesCurveArcLengthRatherThanCartesianChord)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(straightLoop(30.0, 4.0)));
  VelocityProfileConfig config = defaultConfig();
  config.max_velocity_mps = 4.0;

  std::vector<CurveSample> path = {
    sampleAt(0.0, 0.0, 0.0),
    sampleAt(2.0, 1.0, 0.0),
  };
  const VelocityProfileResult result = assignVelocityProfile(
    path, 4.0, 0.0, 2.0, PlannerIntent::FOLLOW_RACING_LINE, reference, config);

  ASSERT_TRUE(result.feasible);
  EXPECT_NEAR(result.traversal_time_s, 0.5, 1e-9);
}

TEST(VelocityProfile, RejectsInvalidInputs)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(straightLoop(20.0, 4.0)));
  VelocityProfileConfig config = defaultConfig();

  // Corrupt accel.
  VelocityProfileConfig bad = config;
  bad.max_accel_mps2 = 0.0;
  std::vector<CurveSample> path = straightPath(3.0, 0.5);
  EXPECT_FALSE(
    assignVelocityProfile(
      path, 1.0, 0.0, 3.0, PlannerIntent::FOLLOW_RACING_LINE, reference, bad).feasible);

  bad = config;
  bad.min_velocity_mps = -0.1;
  EXPECT_FALSE(
    assignVelocityProfile(
      path, 1.0, 0.0, 3.0, PlannerIntent::FOLLOW_RACING_LINE, reference, bad).feasible);

  bad = config;
  bad.min_velocity_mps = bad.max_velocity_mps + 1.0;
  EXPECT_FALSE(
    assignVelocityProfile(
      path, 1.0, 0.0, 3.0, PlannerIntent::FOLLOW_RACING_LINE, reference, bad).feasible);

  // Non-finite start.
  path = straightPath(3.0, 0.5);
  EXPECT_FALSE(
    assignVelocityProfile(
      path, std::numeric_limits<double>::quiet_NaN(), 0.0, 3.0,
      PlannerIntent::FOLLOW_RACING_LINE, reference, config).feasible);

  // Degenerate zero-length path.
  path = {
    sampleAt(0.0, 0.0, 0.0),
    sampleAt(0.0, 0.0, 0.0),
  };
  EXPECT_FALSE(
    assignVelocityProfile(
      path, 1.0, 0.0, 0.0, PlannerIntent::FOLLOW_RACING_LINE, reference, config).feasible);

  // Non-monotonic authoritative arc length.
  path = {
    sampleAt(0.0, 0.0, 0.0),
    sampleAt(1.0, 1.0, 0.0),
    sampleAt(0.5, 2.0, 0.0),
  };
  EXPECT_FALSE(
    assignVelocityProfile(
      path, 1.0, 0.0, 2.0, PlannerIntent::FOLLOW_RACING_LINE, reference, config).feasible);

  // Invalid reference.
  RacelineReference empty;
  path = straightPath(3.0, 0.5);
  EXPECT_FALSE(
    assignVelocityProfile(
      path, 1.0, 0.0, 3.0, PlannerIntent::FOLLOW_RACING_LINE, empty, config).feasible);
}

TEST(VelocityProfile, ZeroAverageSpeedRejected)
{
  RacelineReference reference;
  // Zero raceline speed with zero start leaves a positive-length segment at v=0.
  ASSERT_TRUE(reference.setRacingLine(straightLoop(20.0, 0.0)));
  VelocityProfileConfig config = defaultConfig();
  config.min_velocity_mps = 0.0;
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;

  std::vector<CurveSample> path = straightPath(2.0, 0.5);
  for (CurveSample & sample : path) {
    sample.speed = 42.0;
  }

  const VelocityProfileResult result = assignVelocityProfile(
    path, 0.0, 0.0, 2.0, PlannerIntent::FOLLOW_RACING_LINE, reference, config);

  EXPECT_FALSE(result.feasible);
  for (const CurveSample & sample : path) {
    EXPECT_NEAR(sample.speed, 42.0, 1e-12);
  }
}

} // namespace local_planning
