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

struct ReferenceGeometrySample
{
  double s = 0.0;
  double s_wrapped = 0.0;
  double x = 0.0;
  double y = 0.0;
  double tangent_x = 0.0;
  double tangent_y = 0.0;
  double normal_x = 0.0;
  double normal_y = 0.0;
  double heading = 0.0;
  double curvature = 0.0;
  double velocity = 0.0;
  int segment_index = 0;
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
  double layer_spacing_m = 1.0;
  double lane_spacing_m = 0.2;
  double max_lateral_offset_m = 1.8;
  double max_path_angle_deg = 50.0;
  double sample_spacing_m = 0.1;
  double max_runtime_ms = 25.0;
  double collision_circle_radius_m = 0.20;
  double front_collision_circle_offset_m = 0.26;
  double soft_inflation_distance_m = 0.18;
  int occupied_threshold = 50;
  double friction_coeff = 1.0;
  double min_velocity_mps = 0.5;
  double max_velocity_mps = 10.0;
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

// Rejection counters for one family of edges (direct quartics or ordinary
// cubics), grouped by the lattice layer the edge tried to reach.
struct EdgeLayerDiagnostics
{
  int destination_layer = 0;
  int angle_pruned = 0;
  int geometry_rejected = 0;
  int collided = 0;
  int out_of_grid = 0;
  int accepted = 0;
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
  // Indexed by destination_layer - 1; quartics can enter every layer, cubics
  // only layers >= 2 (their layer-1 entry stays all-zero).
  std::vector<EdgeLayerDiagnostics> direct_quartic_diagnostics;
  std::vector<EdgeLayerDiagnostics> cubic_edge_diagnostics;
  // How many lanes ended up reachable at each layer (index = layer - 1).  The
  // first zero entry is where the search chain broke.
  std::vector<int> reachable_lanes_by_layer;
  double direct_quartic_runtime_ms = 0.0;
  // Layer at which the selected path's direct quartic enters the lattice.
  int direct_entry_layer = -1;
  // Layer the selected path ends at.  Equal to the layer count on a
  // full-horizon path; smaller when the partial-horizon fallback selected the
  // deepest reachable layer because the final layer had no reachable lane.
  int selected_final_layer = -1;
};

} // namespace local_planning

#endif // PLANNING_TYPES_HPP
