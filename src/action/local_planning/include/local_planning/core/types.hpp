#ifndef LOCAL_PLANNING_CORE_TYPES_HPP
#define LOCAL_PLANNING_CORE_TYPES_HPP

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
  double s = 0.0;
  double d = 0.0;
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
  // Last pure-pursuit command if fresh within steering_command_timeout_s; else 0.
  double steering_angle = 0.0;
};

struct OccupancyGrid
{
  std::vector<int8_t> data;   //  -1 (unknown), otherwise 0 to 100 for p(occupied)
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  Point origin;
  // Euclidean distance transform: meters to nearest occupied cell.
  std::vector<float> obstacle_distance_m;
  bool has_euclidean_transform = false;
};

// What the tactical layer asks the planner to do this cycle.  Nothing
// serializes it today; Phase 6 mirrors it as uint8 constants in
// PlannerDecision.msg, and those must then be kept in step.
enum class PlannerIntent : uint8_t
{
  FOLLOW_RACING_LINE = 0,
  OVERTAKE = 1,
  PASS = 2,
  MERGE = 3
};

std::string intentToString(PlannerIntent intent);

// Purely longitudinal: the boundaries are gaps measured with deltaS, and they
// absorb both vehicles' extents rather than modelling either one.
enum class RelativePosition : uint8_t
{
  NONE = 0,
  BEHIND = 1,
  OVERLAPPING = 2,
  AHEAD_NOT_CLEAR = 3,
  AHEAD_AND_CLEAR = 4
};

std::string relativePositionToString(RelativePosition position);

// The complete vehicle state a curve is anchored to at one end.  Two of these
// fully determine a G2 connection: position, tangent, and curvature agree at
// the join, and the speed rides along for the velocity profile.
struct BoundaryState
{
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;    // rad, world frame
  double curvature = 0.0;  // 1/m, positive left
  double speed = 0.0;      // m/s
};

// The curve generator exposes why a requested G2 connection could not be
// built.  Maneuver sampling simply omits an invalid path.
enum class RejectReason : uint8_t
{
  NONE = 0,
  SOLVER_FAILED = 1,
  CURVATURE_LIMIT = 2,
  ARC_LENGTH_LIMIT = 3
};

// One dense sample of a generated curve.  The units the velocity profiler and
// collision checker both consume.
struct CurveSample
{
  double s = 0.0;          // arc length from the start of the candidate, m
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;    // rad
  double curvature = 0.0;  // 1/m
  double speed = 0.0;      // m/s, filled by the velocity profile
};

struct LocalPlannerConfig
{
  // Two-circle footprint, rear circle at the path point and front circle
  // pushed forward along the heading.
  double collision_circle_radius_m = 0.20;
  double front_collision_circle_offset_m = 0.26;
  double soft_inflation_distance_m = 0.18;
  int occupied_threshold = 50;

  // Lateral vehicle extent used as the shared "on the raceline" band for side
  // commit (PASS) and merge-done checks.
  double vehicleWidthM() const {return 2.0 * collision_circle_radius_m;}

  double friction_coeff = 1.0;
  double min_velocity_mps = 0.0;
  double max_velocity_mps = 10.0;
  double max_accel_mps2 = 5.0;
  double max_decel_mps2 = 5.0;
  // Interior/terminal racing-speed multiplier for OVERTAKE/PASS, and MERGE
  // interiors only. MERGE's horizon terminal stays at unscaled raceline speed.
  double overtake_speed_scale = 1.1;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_CORE_TYPES_HPP
