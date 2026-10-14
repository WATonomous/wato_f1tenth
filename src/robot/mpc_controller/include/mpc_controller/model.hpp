#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include "mpc_config.hpp"

namespace mpc {
struct Pose {double x = 0, y = 0, yaw = 0;};
struct Command {double speed = 0, steering = 0;};
using State = std::array<double, 5>;

inline double angleDifference(double a, double b)
{return std::atan2(std::sin(a-b), std::cos(a-b));}

inline bool finite(Pose p)
{return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);}

inline bool validCommand(Command u)
{
  return std::isfinite(u.speed) && std::isfinite(u.steering) &&
         u.speed >= 0 && u.speed <= speed_cap &&
         u.steering >= steering_min && u.steering <= steering_max;
}

inline Pose toLocal(Pose p, Pose frame)
{
  const double c = std::cos(frame.yaw), s = std::sin(frame.yaw);
  return {c*(p.x-frame.x)+s*(p.y-frame.y), -s*(p.x-frame.x)+c*(p.y-frame.y),
    angleDifference(p.yaw, frame.yaw)};
}

inline Pose toMap(Pose p, Pose frame)
{
  const double c = std::cos(frame.yaw), s = std::sin(frame.yaw);
  return {frame.x+c*p.x-s*p.y, frame.y+s*p.x+c*p.y, frame.yaw+p.yaw};
}

inline Pose step(Pose p, Command u, double duration)
{
  if (!finite(p) || !validCommand(u) || !std::isfinite(duration) || duration < 0) {
    throw std::invalid_argument("Invalid bicycle rollout input");
  }
  const double turn = u.speed*std::tan(u.steering)/wheelbase*duration;
  // Mid-heading and sinc avoid cancellation in the straight-line limit.
  const double half = turn*.5;
  const double distance = u.speed*duration*(std::abs(half) < 1e-8 ?
    1-half*half/6 : std::sin(half)/half);
  return {p.x+distance*std::cos(p.yaw+half), p.y+distance*std::sin(p.yaw+half),
    p.yaw+turn};
}

inline double coveringRadius()
{return std::hypot((front_extent+rear_extent)/2, half_width);}
inline Pose coveringCenter(Pose rear)
{
  const double offset=(front_extent-rear_extent)/2;
  return {rear.x+offset*std::cos(rear.yaw),rear.y+offset*std::sin(rear.yaw),rear.yaw};
}
}  // namespace mpc
