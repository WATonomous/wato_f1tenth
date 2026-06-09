#ifndef PLANNING_TYPES_HPP
#define PLANNING_TYPES_HPP

#include <cstdint>
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
  double s;
  double d;
};

struct Odometry
{
  Point position;
  double velocity;
  double heading;
};

struct OccupancyGrid
{
  std::vector<int8_t> data;   //  -1 (unknown), otherwise 0 to 100 for p(occupied)
  int width;
  int height;
  double resolution;
  Point origin;
  std::vector<uint8_t> definitely_blocked_mask;
  std::vector<uint8_t> needs_exact_check_mask;
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
  int max_lane_jump_per_layer = 3;
  double max_path_angle_deg = 50.0;
  double sample_spacing_m = 0.1;
  double max_runtime_ms = 25.0;
  double heuristic_weight = 1.0;
  std::vector<double> heading_buckets_deg{-10.0, -5.0, 0.0, 5.0, 10.0};
  int max_heading_jump_per_layer = 1;
  double max_heading_mismatch_deg = 25.0;
  int heuristic_sample_count = 8;
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
};

struct LocalFrenetPlan
{
  std::vector<Point> path;
};

} // namespace local_planning

#endif // PLANNING_TYPES_HPP
