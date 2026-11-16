// Unit tests for the Stanley control law (no ROS graph needed)

#include <cmath>
#include <limits>
#include <vector>

#include <catch2/catch_all.hpp>

#include "stanley.hpp"

using Catch::Approx;

namespace
{

geometry_msgs::msg::PoseStamped pose_at(double x, double y, double speed)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = speed;  // the global planner stores the raceline speed in z
  return p;
}

// A straight path from the origin along `heading`, `count` points `spacing` apart
nav_msgs::msg::Path straight_path(double heading = 0.0, double spacing = 0.2, int count = 50, double speed = 2.0)
{
  nav_msgs::msg::Path path;
  for (int i = 0; i < count; ++i) {
    path.poses.push_back(pose_at(i * spacing * std::cos(heading), i * spacing * std::sin(heading), speed));
  }
  return path;
}

// A counter-clockwise circle; the last point repeats the first, like a closed raceline
nav_msgs::msg::Path closed_circle(double radius = 5.0, int count = 100)
{
  nav_msgs::msg::Path path;
  for (int i = 0; i <= count; ++i) {
    const double a = 2 * M_PI * (i % count) / count;
    path.poses.push_back(pose_at(radius * std::cos(a), radius * std::sin(a), 2.0));
  }
  return path;
}

stanley::Params params()
{
  stanley::Params p;
  p.k_h = 0.75;
  p.k_e = 0.2;
  p.k_soft = 0.5;
  p.max_steering_angle = 0.52;
  p.wheelbase = 0.324;
  p.speed_limit_active = true;
  p.speed_limit = 5.0;
  p.speed_lookahead_s = 0.0;
  return p;
}

stanley::State state(double x, double y, double yaw, double speed = 2.0)
{
  return stanley::State{x, y, yaw, speed};
}

// What the node's search finds: the waypoint closest to the front axle
size_t closest(const nav_msgs::msg::Path & path, const stanley::State & s, const stanley::Params & p)
{
  double fx, fy;
  stanley::front_axle(s, p.wheelbase, fx, fy);
  size_t best = 0;
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double d = std::hypot(path.poses[i].pose.position.x - fx, path.poses[i].pose.position.y - fy);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

stanley::Result run(const nav_msgs::msg::Path & path, const stanley::State & s, const stanley::Params & p = params())
{
  return stanley::compute(s, path, closest(path, s, p), p);
}

}  // namespace

TEST_CASE("on the line and facing along it: no steering", "[stanley]")
{
  const auto r = run(straight_path(), state(1.0, 0.0, 0.0));
  REQUIRE(r.cte == Approx(0.0).margin(1e-9));
  REQUIRE(r.heading_error == Approx(0.0).margin(1e-9));
  REQUIRE(r.steering == Approx(0.0).margin(1e-9));
  REQUIRE(r.speed == Approx(2.0));
}

TEST_CASE("angles wrap to [-pi, pi]", "[stanley]")
{
  REQUIRE(stanley::wrap_angle(3 * M_PI / 2) == Approx(-M_PI / 2));
  REQUIRE(stanley::wrap_angle(-3 * M_PI / 2) == Approx(M_PI / 2));
  REQUIRE(stanley::wrap_angle(M_PI / 4) == Approx(M_PI / 4));
  REQUIRE(stanley::wrap_angle(5 * M_PI) == Approx(M_PI).margin(1e-9));

  // path heading -179 deg, car yaw +179 deg: a 2 deg error, not -358 deg
  const double deg = M_PI / 180.0;
  const auto path = straight_path(-179 * deg);
  const auto r = run(path, state(path.poses[10].pose.position.x, path.poses[10].pose.position.y, 179 * deg));
  REQUIRE(r.heading_error == Approx(2 * deg).margin(1e-9));
  REQUIRE(std::abs(r.steering) < 0.1);
}

TEST_CASE("cross-track error sign: right of the path steers left", "[stanley]")
{
  const auto path = straight_path();  // along +x, so +y is left

  const auto right = run(path, state(2.0, -0.5, 0.0));
  REQUIRE(right.cte == Approx(0.5));
  REQUIRE(right.steering > 0.0);  // turns left, back onto the line

  const auto left = run(path, state(2.0, 0.5, 0.0));
  REQUIRE(left.cte == Approx(-0.5));
  REQUIRE(left.steering < 0.0);
}

TEST_CASE("steering and speed stay within the limits", "[stanley]")
{
  const auto p = params();

  // far off and pointing away from the path: both terms push the same way, past the limit
  const auto away_right = run(straight_path(), state(2.0, -5.0, -M_PI / 2));
  REQUIRE(away_right.steering == Approx(p.max_steering_angle));
  const auto away_left = run(straight_path(), state(2.0, 5.0, M_PI / 2));
  REQUIRE(away_left.steering == Approx(-p.max_steering_angle));

  // path speed above the cap
  const auto fast = straight_path(0.0, 0.2, 50, 8.0);
  REQUIRE(run(fast, state(1.0, 0.0, 0.0)).speed == Approx(p.speed_limit));

  // cap off: the path speed is used as is
  auto uncapped = p;
  uncapped.speed_limit_active = false;
  REQUIRE(run(fast, state(1.0, 0.0, 0.0), uncapped).speed == Approx(8.0));

  // negative path speed is clamped to 0 when the cap is on
  const auto backwards = straight_path(0.0, 0.2, 50, -1.0);
  REQUIRE(run(backwards, state(1.0, 0.0, 0.0)).speed == Approx(0.0));
}

TEST_CASE("duplicate path points are dropped", "[stanley]")
{
  // a doubled point in the middle, and a closing point that repeats the first
  std::vector<geometry_msgs::msg::PoseStamped> poses = {
    pose_at(0, 0, 1), pose_at(0, 0, 1), pose_at(1, 0, 1), pose_at(2, 0, 1),
    pose_at(2, 0, 1), pose_at(3, 0, 1), pose_at(0, 0, 1)};
  stanley::drop_duplicate_points(poses);
  REQUIRE(poses.size() == 4);
  REQUIRE(poses.front().pose.position.x == Approx(0.0));
  REQUIRE(poses.back().pose.position.x == Approx(3.0));

  // a closed loop: no full-lock steering at the seam (the once-per-lap bug)
  auto loop = closed_circle();
  stanley::drop_duplicate_points(loop.poses);
  REQUIRE(loop.poses.size() == 100);
  const auto n = loop.poses.size();
  for (size_t i : {n - 3, n - 2, n - 1, size_t{0}, size_t{1}, size_t{2}}) {
    const auto & w = loop.poses[i].pose.position;
    const double tangent = std::atan2(w.x, -w.y);  // counter-clockwise
    const auto r = run(loop, state(w.x, w.y, tangent));
    INFO("waypoint " << i);
    REQUIRE(std::abs(r.steering) < 0.1);
  }
}

TEST_CASE("cross-track error is the distance to the line, not to a waypoint", "[stanley]")
{
  // car 0.3 m right of a straight path with 0.5 m spacing (like the car's raceline),
  // angled 0.3 rad toward it, moved along across several waypoints: measuring to the
  // nearest waypoint would saw-tooth here, the distance to the line must not
  const auto path = straight_path(0.0, 0.5, 30);
  const auto p = params();
  const double yaw = 0.3;
  for (double fx = 3.0; fx <= 6.0; fx += 0.05) {
    // place the base so the front axle is at (fx, -0.3)
    const auto s = state(fx - p.wheelbase * std::cos(yaw), -0.3 - p.wheelbase * std::sin(yaw), yaw);
    const auto r = run(path, s, p);
    INFO("front axle x = " << fx);
    REQUIRE(r.cte == Approx(0.3).margin(1e-9));
    REQUIRE(r.heading_error == Approx(-yaw).margin(1e-9));
  }
}

TEST_CASE("speed look-ahead brakes before a slow section", "[stanley]")
{
  // straight path, 0.5 m spacing: 4 m/s until x = 5 m, then 2 m/s
  nav_msgs::msg::Path path;
  for (int i = 0; i < 30; ++i) {
    path.poses.push_back(pose_at(0.5 * i, 0.0, 0.5 * i < 5.0 ? 4.0 : 2.0));
  }
  auto p = params();
  const auto s = state(1.0, 0.0, 0.0, 4.0);  // front axle at x = 1.324, 3.7 m before the slow part

  // off: the speed of the closest waypoint
  REQUIRE(run(path, s, p).speed == Approx(4.0));

  // 0.5 s at 4 m/s = 2 m ahead: still all fast
  p.speed_lookahead_s = 0.5;
  REQUIRE(run(path, s, p).speed == Approx(4.0));

  // 1 s at 4 m/s = 4 m ahead: reaches the slow part, so brake now
  p.speed_lookahead_s = 1.0;
  REQUIRE(run(path, s, p).speed == Approx(2.0));

  // standing still looks nowhere ahead
  REQUIRE(run(path, state(1.0, 0.0, 0.0, 0.0), p).speed == Approx(4.0));
}
