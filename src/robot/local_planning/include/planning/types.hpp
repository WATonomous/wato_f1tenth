#ifndef PLANNING_TYPES_HPP
#define PLANNING_TYPES_HPP

#include <cstdint>
#include <chrono>
#include <string>
#include <vector>

namespace local_planning
{

struct Point
{
  double x;
  double y;
  double velocity;

  Point(double x = 0.0, double y = 0.0, double velocity = 0.0)
  : x(x), y(y), velocity(velocity) {}
};

struct FrenetPoint
{
  double s = 0.0;
  double d = 0.0;
  double slope = 0.0;
  // d²d/ds².  This is the quintic boundary curvature in Frenet coordinates.
  double second_derivative = 0.0;
};

struct Odometry
{
  Point position;
  double velocity;
  double heading;
  double steering_angle = 0.0;
  bool has_steering_angle = false;
};

struct OccupancyGrid
{
  std::vector<int8_t> data;   //  -1 (unknown), otherwise 0 to 100 for p(occupied)
  int width;
  int height;
  double resolution;
  Point origin;
  std::vector<float> obstacle_distance_m;
  bool has_clearance_cache = false;
};

enum class LocalPlannerIntent : uint8_t
{
  FOLLOW_RACING_LINE = 0,
  OVERTAKE = 1,
  MERGE = 2
};

std::string intentToString(LocalPlannerIntent intent);

//for explanations see the yaml
struct LocalFrenetPlannerConfig
{
  double horizon_m = 6.0;
  double layer_spacing_m = 0.5;
  double lane_spacing_m = 0.1;
  double max_lateral_offset_m = 1.8;
  double max_path_angle_deg = 50.0;
  double sample_spacing_m = 0.1;
  double max_runtime_ms = 25.0;
  double collision_circle_radius_m = 0.20;
  double front_collision_circle_offset_m = 0.26;
  double soft_inflation_distance_m = 0.18;
  double soft_inflation_cost = 100.0;
  int occupied_threshold = 50;
  double friction_coeff = 1.0;
  double min_velocity_mps = 0.5;
  double max_velocity_mps = 10.0;
  double time_weight = 1.0;
  double curvature_change_weight = 0.4;
  double follow_d_weight = 0.20;
  double overtake_d_weight = 0.02;
  double merge_d_weight = 0.20;
  double merge_terminal_d_weight = 0.0;
  bool angle_smoothing_enabled = false;
  bool velocity_smoothing_enabled = false;
  double velocity_smoothing_max_accel_mps2 = 2.5;
  double velocity_smoothing_max_decel_mps2 = 2.5;
  double wheelbase_m = 0.33;
  double steering_command_timeout_s = 0.06;
};

struct LocalFrenetPlan
{
  enum class Status : uint8_t
  {
    SUCCESS,
    INVALID_REFERENCE,
    NO_PATH,
    DEADLINE_EXCEEDED
  };

  std::vector<Point> path;
  Status status = Status::NO_PATH;
};

} // namespace local_planning

#endif // PLANNING_TYPES_HPP
