#include <gtest/gtest.h>

#include "local_planning/curves/frenet_connection_generator.hpp"
#include "local_planning/curves/frenet_polynomial.hpp"
#include "local_planning/reference/raceline_reference.hpp"
#include "local_planning/reference/reference_window.hpp"

#include <cmath>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// A circle is the one reference whose Frenet geometry has a closed form:
// kappa = 1/R exactly, kappa' = 0 exactly.  Every analytic claim in the port is
// checkable against it without a tolerance that hides an error.
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

RacelineReference makeCircle(double radius = 10.0, int count = 720)
{
  RacelineReference reference;
  EXPECT_TRUE(reference.setRacingLine(circleLine(radius, count)));
  return reference;
}

FrenetPolynomial constantOffset(double d, double delta_s)
{
  return computeQuintic(d, 0.0, 0.0, d, 0.0, 0.0, delta_s);
}

}  // namespace

// --- The polynomial itself -------------------------------------------------

TEST(FrenetPolynomial, MatchedBoundariesGiveTheSmoothstepQuintic)
{
  const double magnitude = 0.55;
  const double length = 6.0;
  const FrenetPolynomial polynomial =
    computeQuintic(0.0, 0.0, 0.0, magnitude, 0.0, 0.0, length);

  for (int i = 0; i <= 100; ++i) {
    const double t = static_cast<double>(i) / 100.0;
    const double expected =
      magnitude * (10.0 * t * t * t - 15.0 * t * t * t * t + 6.0 * t * t * t * t * t);
    EXPECT_NEAR(polynomial.evaluate(t), expected, 1e-12) << "t=" << t;
  }
}

TEST(FrenetPolynomial, HonoursEveryBoundaryConditionExactly)
{
  const double length = 4.0;
  const FrenetPolynomial polynomial =
    computeQuintic(0.10, 0.05, -0.02, 0.70, -0.03, 0.01, length);

  EXPECT_NEAR(polynomial.evaluate(0.0), 0.10, 1e-12);
  EXPECT_NEAR(polynomial.evaluateDerivative(0.0) / length, 0.05, 1e-12);
  EXPECT_NEAR(polynomial.evaluateSecondDerivative(0.0) / (length * length), -0.02, 1e-12);
  EXPECT_NEAR(polynomial.evaluate(1.0), 0.70, 1e-12);
  EXPECT_NEAR(polynomial.evaluateDerivative(1.0) / length, -0.03, 1e-12);
  EXPECT_NEAR(polynomial.evaluateSecondDerivative(1.0) / (length * length), 0.01, 1e-12);
}

// The property the whole overshoot argument rests on.  A G2 clothoid pinned at
// both ends swings wide to satisfy the terminal curvature; this cannot.
TEST(FrenetPolynomial, MatchedBoundariesNeverExceedTheTargetOffset)
{
  for (const double magnitude : {0.30, 0.55, 0.75, -0.55}) {
    const FrenetPolynomial polynomial =
      computeQuintic(0.0, 0.0, 0.0, magnitude, 0.0, 0.0, 6.0);
    double previous = 0.0;
    for (int i = 0; i <= 200; ++i) {
      const double t = static_cast<double>(i) / 200.0;
      const double value = polynomial.evaluate(t);
      EXPECT_LE(std::abs(value), std::abs(magnitude) + 1e-12) << "t=" << t;
      // Strictly monotone: d'(t) = 30 D t^2 (1-t)^2 never changes sign.
      EXPECT_GE(value * (magnitude > 0.0 ? 1.0 : -1.0),
        previous * (magnitude > 0.0 ? 1.0 : -1.0) - 1e-12);
      previous = value;
    }
    EXPECT_NEAR(polynomial.evaluate(1.0), magnitude, 1e-12);
  }
}

TEST(FrenetPolynomial, VehicleCurvatureInversionRoundTrips)
{
  const double reference_curvature = 0.2;
  const double reference_curvature_derivative = 0.03;
  const double d = 0.4;
  const double d_prime = 0.15;
  const double vehicle_curvature = 0.9;

  const double d_double_prime = frenetSecondDerivativeForVehicleCurvature(
    vehicle_curvature, d, d_prime, reference_curvature, reference_curvature_derivative);

  // Feed it back through the forward formula the sampler uses.
  const double tangent_scale = 1.0 - reference_curvature * d;
  const double normal_term = reference_curvature * tangent_scale + d_double_prime;
  const double tangential_term =
    -reference_curvature_derivative * d - 2.0 * reference_curvature * d_prime;
  const double norm_sq = tangent_scale * tangent_scale + d_prime * d_prime;
  const double recovered =
    (tangent_scale * normal_term - d_prime * tangential_term) / (norm_sq * std::sqrt(norm_sq));
  EXPECT_NEAR(recovered, vehicle_curvature, 1e-12);
}

// --- The reference the sampler runs against --------------------------------

TEST(ReferenceWindow, CoversTheRequestedSpanInclusiveOfBothEnds)
{
  const RacelineReference reference = makeCircle();
  ReferenceWindow window;
  ASSERT_TRUE(window.build(reference, 4.0, 6.0, 0.1));

  EXPECT_EQ(window.size(), 61u);
  EXPECT_DOUBLE_EQ(window.startS(), 4.0);
  EXPECT_NEAR(window.spacingM(), 0.1, 1e-12);
  EXPECT_NEAR(window.lengthM(), 6.0, 1e-12);
  EXPECT_NEAR(window.sAt(0), 4.0, 1e-12);
  EXPECT_NEAR(window.sAt(window.size() - 1), 10.0, 1e-12);
  EXPECT_EQ(window.indexForS(4.0), 0u);
  EXPECT_EQ(window.indexForS(10.0), window.size() - 1);
  EXPECT_EQ(window.indexForS(7.0), 30u);
  // Out of span clamps rather than wrapping into nonsense.  Behind the window
  // clamps to the start, ahead of it to the end -- and "behind" is measured
  // with deltaS, so more than half a lap ahead reads as behind.
  EXPECT_EQ(window.indexForS(2.0), 0u);
  EXPECT_EQ(window.indexForS(12.0), window.size() - 1);
}

TEST(ReferenceWindow, IndexLookupSurvivesTheLoopSeam)
{
  const RacelineReference reference = makeCircle();
  const double length = reference.totalLength();
  ReferenceWindow window;
  // Start 3 m before the seam so the window straddles s = 0.
  ASSERT_TRUE(window.build(reference, length - 3.0, 6.0, 0.1));
  EXPECT_EQ(window.indexForS(reference.wrapS(length - 3.0)), 0u);
  EXPECT_EQ(window.indexForS(1.0), 40u);
}

TEST(RacelineReference, CurvatureDerivativeIsZeroOnACircle)
{
  const RacelineReference reference = makeCircle(10.0, 720);
  for (double s = 0.0; s < 60.0; s += 1.7) {
    const ReferenceGeometrySample sample = reference.sampleAtS(s);
    EXPECT_NEAR(sample.curvature, 0.1, 1e-4) << "s=" << s;
    EXPECT_NEAR(sample.curvature_derivative, 0.0, 1e-4) << "s=" << s;
  }
}

// --- The sampler -----------------------------------------------------------

// The exact check the old three-point curvature estimate could not pass.
TEST(FrenetConnectionGenerator, ConstantOffsetCurvatureMatchesTheClosedForm)
{
  const double radius = 10.0;
  const RacelineReference reference = makeCircle(radius);
  ReferenceWindow window;
  ASSERT_TRUE(window.build(reference, 0.0, 6.0, 0.1));

  const FrenetConnectionGenerator generator;
  for (const double d : {-0.75, -0.30, 0.0, 0.55, 0.75}) {
    std::vector<CurveSample> path;
    const auto result = generator.generate(
      window, 0, window.size() - 1, constantOffset(d, window.lengthM()), path);
    ASSERT_TRUE(result.valid) << "d=" << d;
    ASSERT_EQ(path.size(), window.size());

    for (std::size_t i = 0; i < path.size(); ++i) {
      const double kappa_ref = window.at(i).curvature;
      const double expected = kappa_ref / (1.0 - d * kappa_ref);
      EXPECT_NEAR(path[i].curvature, expected, 1e-12) << "d=" << d << " i=" << i;
      EXPECT_NEAR(path[i].d, d, 1e-12);
      // d' = 0, so the path heading is the reference heading exactly.
      EXPECT_NEAR(path[i].heading, window.at(i).heading, 1e-12);
    }
  }
}

TEST(FrenetConnectionGenerator, SamplesCarryTheirExactStationAndOffset)
{
  const RacelineReference reference = makeCircle();
  ReferenceWindow window;
  ASSERT_TRUE(window.build(reference, 2.0, 6.0, 0.1));

  const FrenetConnectionGenerator generator;
  std::vector<CurveSample> path;
  const FrenetPolynomial polynomial =
    computeQuintic(0.0, 0.0, 0.0, 0.55, 0.0, 0.0, window.lengthM());
  ASSERT_TRUE(generator.generate(window, 0, window.size() - 1, polynomial, path).valid);

  for (std::size_t i = 0; i < path.size(); ++i) {
    const ReferenceGeometrySample & sample = window.at(i);
    EXPECT_DOUBLE_EQ(path[i].raceline_s, sample.s_wrapped);
    // The offset the sample reports must be the offset it is actually at.
    const double dx = path[i].x - sample.x;
    const double dy = path[i].y - sample.y;
    EXPECT_NEAR(dx * sample.normal_x + dy * sample.normal_y, path[i].d, 1e-12);
  }
}

TEST(FrenetConnectionGenerator, ArcLengthIsMonotoneAndMatchesTheChordSum)
{
  const RacelineReference reference = makeCircle();
  ReferenceWindow window;
  ASSERT_TRUE(window.build(reference, 0.0, 6.0, 0.02));

  const FrenetConnectionGenerator generator;
  std::vector<CurveSample> path;
  const FrenetPolynomial polynomial =
    computeQuintic(0.0, 0.0, 0.0, 0.55, 0.0, 0.0, window.lengthM());
  ASSERT_TRUE(generator.generate(window, 0, window.size() - 1, polynomial, path).valid);

  EXPECT_DOUBLE_EQ(path.front().s, 0.0);
  double chord_sum = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_GT(path[i].s, path[i - 1].s) << "i=" << i;
    chord_sum += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  // Not exact, and cannot be: the reference spline is parameterised by
  // cumulative *chord* length, so |r'(s)| is only approximately 1.  On this
  // 720-point circle that is a ~3e-6 relative bias, which is what separates the
  // integrated differential from the chord sum.  Both are far under the
  // resolution of anything downstream -- the velocity profile only differences
  // s over 0.1 m steps.
  EXPECT_NEAR(path.back().s, chord_sum, 1e-4);
}

// A connection followed by a constant-offset tail is C2 at the join by
// construction: both sides have d' = d'' = 0 at the same d.  This is what makes
// the old matches() continuity gate unnecessary rather than merely loose.
TEST(FrenetConnectionGenerator, TailJoinsTheTransitionWithoutADiscontinuity)
{
  const RacelineReference reference = makeCircle();
  ReferenceWindow window;
  ASSERT_TRUE(window.build(reference, 0.0, 6.0, 0.1));
  const std::size_t join = 30;   // s = 3.0
  const double d = 0.55;

  const FrenetConnectionGenerator generator;
  const double transition_length = window.spacingM() * static_cast<double>(join);
  const double tail_length =
    window.spacingM() * static_cast<double>(window.size() - 1 - join);

  // Sample the two legs independently, so the join station appears in both and
  // the comparison is like for like.  Comparing consecutive samples of a
  // concatenated path would measure the reference turning between stations, not
  // continuity.
  std::vector<CurveSample> transition;
  ASSERT_TRUE(generator.generate(
      window, 0, join, computeQuintic(0.0, 0.0, 0.0, d, 0.0, 0.0, transition_length),
      transition).valid);
  std::vector<CurveSample> tail;
  ASSERT_TRUE(generator.generate(
      window, join, window.size() - 1, constantOffset(d, tail_length), tail).valid);

  // C2 at the join by construction: the transition ends with d' = d'' = 0 at
  // this same d, which is exactly what the tail's boundaries request.  This is
  // what makes the old matches() continuity gate unnecessary rather than merely
  // loose, so it is asserted to machine precision.
  EXPECT_NEAR(transition.back().d, tail.front().d, 1e-12);
  EXPECT_NEAR(transition.back().x, tail.front().x, 1e-12);
  EXPECT_NEAR(transition.back().y, tail.front().y, 1e-12);
  EXPECT_NEAR(transition.back().heading, tail.front().heading, 1e-12);
  EXPECT_NEAR(transition.back().curvature, tail.front().curvature, 1e-12);
  EXPECT_NEAR(
    transition.back().curvature,
    window.at(join).curvature / (1.0 - d * window.at(join).curvature), 1e-12);

  // Concatenating drops the duplicated join sample and holds d flat afterwards.
  std::vector<CurveSample> path = transition;
  ASSERT_TRUE(generator.generate(
      window, join, window.size() - 1, constantOffset(d, tail_length), path).valid);
  ASSERT_EQ(path.size(), window.size());
  for (std::size_t i = join; i < path.size(); ++i) {
    EXPECT_NEAR(path[i].d, d, 1e-12) << "i=" << i;
  }
}

TEST(FrenetConnectionGenerator, RejectsCurvatureBeyondTheSteeringLimit)
{
  const RacelineReference reference = makeCircle();
  ReferenceWindow window;
  ASSERT_TRUE(window.build(reference, 0.0, 6.0, 0.1));

  const FrenetConnectionGenerator generator;
  std::vector<CurveSample> path;
  // 0.75 m of lateral travel inside 0.5 m of station needs kappa far past 1.74.
  const FrenetPolynomial polynomial = computeQuintic(0.0, 0.0, 0.0, 0.75, 0.0, 0.0, 0.5);
  const auto result = generator.generate(window, 0, 5, polynomial, path);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::CURVATURE_LIMIT);
  EXPECT_TRUE(path.empty()) << "a rejected connection must leave no samples behind";
}

TEST(FrenetConnectionGenerator, RejectsAHeadingBeyondTheLimit)
{
  const RacelineReference reference = makeCircle();
  ReferenceWindow window;
  ASSERT_TRUE(window.build(reference, 0.0, 6.0, 0.1));

  FrenetConnectionConfig config;
  config.max_curvature_inv_m = 1e9;    // isolate the heading test
  config.max_path_angle_deg = 30.0;
  const FrenetConnectionGenerator generator(config);
  std::vector<CurveSample> path;
  // A start slope of 1.0 is 45 degrees off the reference tangent.
  const FrenetPolynomial polynomial = computeQuintic(0.0, 1.0, 0.0, 0.55, 0.0, 0.0, 6.0);
  const auto result = generator.generate(window, 0, window.size() - 1, polynomial, path);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::HEADING_LIMIT);
  EXPECT_TRUE(path.empty());
}

TEST(FrenetConnectionGenerator, RejectsAnOffsetAtTheCentreOfCurvature)
{
  // Radius 1.0, so d = 1.0 is exactly the centre and A = 1 - kappa*d = 0.
  const RacelineReference reference = makeCircle(1.0, 360);
  ReferenceWindow window;
  ASSERT_TRUE(window.build(reference, 0.0, 1.0, 0.1));

  FrenetConnectionConfig config;
  config.max_curvature_inv_m = 1e9;
  const FrenetConnectionGenerator generator(config);
  std::vector<CurveSample> path;
  const auto result = generator.generate(
    window, 0, window.size() - 1, constantOffset(1.0, window.lengthM()), path);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::CHART_SINGULAR);
  EXPECT_TRUE(path.empty());
}

TEST(FrenetConnectionGenerator, RejectsADegenerateSpan)
{
  const RacelineReference reference = makeCircle();
  ReferenceWindow window;
  ASSERT_TRUE(window.build(reference, 0.0, 6.0, 0.1));

  const FrenetConnectionGenerator generator;
  std::vector<CurveSample> path;
  EXPECT_FALSE(generator.generate(window, 10, 10, constantOffset(0.0, 1.0), path).valid);
  EXPECT_FALSE(generator.generate(window, 20, 10, constantOffset(0.0, 1.0), path).valid);
  EXPECT_TRUE(path.empty());
}

}  // namespace local_planning
