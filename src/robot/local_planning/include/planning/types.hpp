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
  double obstacle_inflation_distance_m = 0.2;
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

//to make my debugging life easy
struct LocalFrenetPlannerDiagnostics
{
  LocalPlannerIntent intent = LocalPlannerIntent::FOLLOW_RACING_LINE;
  FrenetPoint ego_frenet{};
  double selected_cost = 0.0;
  double selected_final_d = 0.0;
  int selected_final_lane = 0;
  int total_lanes = 0;
  int layers = 0;
  int attempted_edges = 0;
  int valid_edges = 0;
  int invalid_collision_edges = 0;
  int invalid_out_of_grid_edges = 0;
  int invalid_geometry_edges = 0;
  double planner_setup_ms = 0.0;
  double planner_search_ms = 0.0;
  double planner_goal_select_ms = 0.0;
  double planner_reconstruct_ms = 0.0;
  std::vector<double> lane_offsets;
};

struct LocalFrenetPlan
{
  std::vector<Point> path;
  LocalFrenetPlannerDiagnostics diagnostics;
};

} // namespace local_planning

#endif // PLANNING_TYPES_HPP
