#include <gtest/gtest.h>

#include "local_planning/curves/curve_connection_generator.hpp"

#include <algorithm>
#include <cmath>

namespace local_planning
{
namespace
{

BoundaryState makeState(
  double x, double y, double heading, double curvature, double speed = 0.0)
{
  BoundaryState state;
  state.x = x;
  state.y = y;
  state.heading = heading;
  state.curvature = curvature;
  state.speed = speed;
  return state;
}

ConnectionRequest makeRequest(const BoundaryState & start, const BoundaryState & terminal)
{
  ConnectionRequest request;
  request.start = start;
  request.terminal = terminal;
  return request;
}

} // namespace

// The strongest available check on the Simpson march: the solver guarantees the
// true curve ends at the requested terminal, so drift shows up as the last
// sample missing it.
TEST(CurveConnectionGenerator, SampledPathArrivesAtRequestedTerminal)
{
  const CurveConnectionGenerator generator;

  struct Case
  {
    const char * name;
    BoundaryState start;
    BoundaryState terminal;
  };

  const Case cases[] = {
    {"straight", makeState(0, 0, 0, 0), makeState(5.0, 0.0, 0.0, 0.0)},
    {"lane shift", makeState(0, 0, 0, 0), makeState(4.0, 0.5, 0.0, 0.0)},
    {"curved start", makeState(0, 0, 0, 0.5), makeState(3.0, 1.2, 0.6, 0.3)},
    {"right shift", makeState(0, 0, 0, -0.2), makeState(4.0, -0.7, -0.1, 0.0)},
    {"heading change", makeState(0, 0, 0, 0), makeState(2.5, 1.0, 0.9, 0.0)},
  };

  for (const Case & c : cases) {
    const GeneratedConnection connection = generator.generate(makeRequest(c.start, c.terminal));
    ASSERT_TRUE(connection.valid) << c.name;
    ASSERT_FALSE(connection.samples.empty()) << c.name;

    const CurveSample & last = connection.samples.back();
    EXPECT_NEAR(last.x, c.terminal.x, 1e-6) << c.name;
    EXPECT_NEAR(last.y, c.terminal.y, 1e-6) << c.name;
    EXPECT_NEAR(last.heading, c.terminal.heading, 1e-9) << c.name;
    EXPECT_NEAR(last.curvature, c.terminal.curvature, 1e-9) << c.name;
  }
}

// A straight connection has a closed form, so it can be checked outright.
TEST(CurveConnectionGenerator, StraightLineMatchesClosedForm)
{
  const CurveConnectionGenerator generator;
  const GeneratedConnection connection =
    generator.generate(makeRequest(makeState(0, 0, 0, 0), makeState(5.0, 0.0, 0.0, 0.0)));

  ASSERT_TRUE(connection.valid);
  for (const CurveSample & sample : connection.samples) {
    EXPECT_NEAR(sample.x, sample.s, 1e-9);
    EXPECT_NEAR(sample.y, 0.0, 1e-9);
    EXPECT_NEAR(sample.heading, 0.0, 1e-12);
    EXPECT_NEAR(sample.curvature, 0.0, 1e-12);
  }
}

// Catches x/y and heading drifting apart, which the endpoint check would miss if
// both drifted together.  They differ even for an exact curve, by a second-order
// term of order dkappa * h^2 / 24 -- hence the loose tolerance, still far below
// any real integration fault, which would also grow along the path.
TEST(CurveConnectionGenerator, ChordDirectionAgreesWithReportedHeading)
{
  const CurveConnectionGenerator generator;
  const GeneratedConnection connection =
    generator.generate(makeRequest(makeState(0, 0, 0, 0.3), makeState(4.0, 1.0, 0.5, -0.2)));

  ASSERT_TRUE(connection.valid);
  ASSERT_GT(connection.samples.size(), 2u);

  for (std::size_t i = 1; i < connection.samples.size(); ++i) {
    const CurveSample & a = connection.samples[i - 1];
    const CurveSample & b = connection.samples[i];
    const double ds = b.s - a.s;
    if (ds < 1e-9) {
      continue;
    }
    const double chord_heading = std::atan2(b.y - a.y, b.x - a.x);
    const double mean_heading = 0.5 * (a.heading + b.heading);
    EXPECT_NEAR(chord_heading, mean_heading, 2e-3) << "at sample " << i;
  }
}

// The collision sweep relies on s being monotone and never stepping further than
// the configured spacing.
TEST(CurveConnectionGenerator, ArcLengthIsMonotoneAndRespectsSpacing)
{
  CurveGeneratorConfig config;
  config.sample_spacing_m = 0.05;
  const CurveConnectionGenerator generator(config);

  const GeneratedConnection connection =
    generator.generate(makeRequest(makeState(0, 0, 0, 0), makeState(4.0, 0.8, 0.2, 0.0)));

  ASSERT_TRUE(connection.valid);
  EXPECT_DOUBLE_EQ(connection.samples.front().s, 0.0);

  for (std::size_t i = 1; i < connection.samples.size(); ++i) {
    const double ds = connection.samples[i].s - connection.samples[i - 1].s;
    EXPECT_GT(ds, 0.0) << "at sample " << i;
    EXPECT_LE(ds, config.sample_spacing_m + 1e-12) << "at sample " << i;
  }
}

// Terminal curvature is a hard boundary condition, not a hint: sweeping it with
// the endpoints fixed must give genuinely different curves, each landing exactly
// on its own request.
TEST(CurveConnectionGenerator, TerminalCurvatureIsHonouredExactly)
{
  const CurveConnectionGenerator generator;
  const BoundaryState start = makeState(0, 0, 0, 0);

  double previous_peak = -1.0;
  for (const double requested : {-0.8, -0.4, 0.0, 0.4, 0.8}) {
    const GeneratedConnection connection =
      generator.generate(makeRequest(start, makeState(4.0, 0.5, 0.0, requested)));
    ASSERT_TRUE(connection.valid) << "kappa = " << requested;

    EXPECT_DOUBLE_EQ(connection.samples.back().curvature, requested);
    EXPECT_DOUBLE_EQ(connection.actual_terminal.curvature, requested);

    double peak = 0.0;
    for (const CurveSample & sample : connection.samples) {
      peak = std::max(peak, std::abs(sample.curvature));
    }
    EXPECT_NE(peak, previous_peak) << "kappa = " << requested;
    previous_peak = peak;
  }
}

// A zero is a real instruction, not an absence of one -- it forces the curve to
// build curvature for the shift and then unwind it.  This is what makes a
// defaulted zero expensive on a corner.
TEST(CurveConnectionGenerator, ZeroTerminalCurvatureForcesTheCurveToUnwind)
{
  const CurveConnectionGenerator generator;
  const GeneratedConnection connection =
    generator.generate(makeRequest(makeState(0, 0, 0, 0), makeState(4.0, 0.5, 0.0, 0.0)));

  ASSERT_TRUE(connection.valid);

  double peak = 0.0;
  for (const CurveSample & sample : connection.samples) {
    peak = std::max(peak, std::abs(sample.curvature));
  }
  EXPECT_GT(peak, 0.1) << "a lateral shift cannot be made without curvature";
  EXPECT_DOUBLE_EQ(connection.samples.front().curvature, 0.0);
  EXPECT_DOUBLE_EQ(connection.samples.back().curvature, 0.0);
}

// Speed is not the curve's business; it rides through for the velocity profile.
TEST(CurveConnectionGenerator, SamplesCarryNoSpeedAndTerminalSpeedPassesThrough)
{
  const CurveConnectionGenerator generator;
  const GeneratedConnection connection =
    generator.generate(makeRequest(makeState(0, 0, 0, 0), makeState(4.0, 0.5, 0.0, 0.0, 7.5)));

  ASSERT_TRUE(connection.valid);
  EXPECT_DOUBLE_EQ(connection.actual_terminal.speed, 7.5);
  for (const CurveSample & sample : connection.samples) {
    EXPECT_DOUBLE_EQ(sample.speed, 0.0);
  }
}

// A curve too tight to drive must come back invalid, not as a path the car
// cannot follow.
TEST(CurveConnectionGenerator, RejectsCurvatureBeyondTheSteeringLimit)
{
  const CurveConnectionGenerator generator;
  const GeneratedConnection connection =
    generator.generate(makeRequest(makeState(0, 0, 0, 0), makeState(0.6, 0.0, 2.6, 0.0)));

  EXPECT_FALSE(connection.valid);
  EXPECT_EQ(connection.reject_reason, RejectReason::CURVATURE_LIMIT);
  EXPECT_TRUE(connection.samples.empty());
}

TEST(CurveConnectionGenerator, RejectsArcLengthBeyondTheLimit)
{
  CurveGeneratorConfig config;
  config.max_arc_length_m = 1.0;   // shorter than any real connection
  const CurveConnectionGenerator generator(config);

  const GeneratedConnection connection =
    generator.generate(makeRequest(makeState(0, 0, 0, 0), makeState(5.0, 0.0, 0.0, 0.0)));

  EXPECT_FALSE(connection.valid);
  EXPECT_EQ(connection.reject_reason, RejectReason::ARC_LENGTH_LIMIT);
  EXPECT_TRUE(connection.samples.empty());
}

// Singular for the solver.  The degenerate zero-length jump-one case belongs to
// the candidate builder and must not arrive here.
TEST(CurveConnectionGenerator, RejectsCoincidentEndpoints)
{
  const CurveConnectionGenerator generator;
  const GeneratedConnection connection =
    generator.generate(makeRequest(makeState(0, 0, 0, 0), makeState(0.0, 0.0, 0.0, 0.0)));

  EXPECT_FALSE(connection.valid);
  EXPECT_EQ(connection.reject_reason, RejectReason::SOLVER_FAILED);
  EXPECT_TRUE(connection.samples.empty());
}

} // namespace local_planning
