#ifndef LOCAL_PLANNING_CORE_TYPES_HPP
#define LOCAL_PLANNING_CORE_TYPES_HPP

#include <cstddef>
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
  std::vector<float> obstacle_distance_m;
  bool has_clearance_cache = false;
};

// What the state manager asks the planner to do this cycle.  Mirrors the
// constants in msg/PlannerIntent.msg; keep the two in step.
enum class PlannerIntent : uint8_t
{
  FOLLOW_RACING_LINE = 0,
  OVERTAKE = 1,
  PASS = 2,
  MERGE = 3
};

std::string intentToString(PlannerIntent intent);

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

// Why a candidate was thrown away.  Every stage that can reject owns a subset:
// the curve generator owns the solver/geometry reasons, the sampler owns the
// topology reasons, collision owns the grid reasons, velocity owns dynamics.
// COUNT is the array size for the Phase 3 per-cycle reject counters.
enum class RejectReason : uint8_t
{
  NONE = 0,
  SOLVER_FAILED = 1,          // curve library did not converge
  CURVATURE_LIMIT = 2,        // exceeds the steering limit somewhere along the arc
  ARC_LENGTH_LIMIT = 3,       // solution loops or doubles back
  TARGET_OUT_OF_TRACK = 4,    // sampled |d| leaves the drivable corridor
  TARGET_WRONG_D_SIGN = 5,    // sampled target violates the intent's d topology
  COLLISION = 6,
  OUT_OF_GRID = 7,
  INFEASIBLE_SPEED = 8,       // no velocity profile satisfies the accel limits
  COUNT = 9
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

// One complete two-jump path.  Samples are a single run rather than two
// vectors — collision and velocity walk the whole path — with `jump2_begin`
// marking the shared boundary the two jumps agree on.
struct Candidate
{
  int id = -1;
  std::vector<CurveSample> samples;
  std::size_t jump2_begin = 0;
  bool valid = false;
  RejectReason reject_reason = RejectReason::NONE;
  double predicted_time_s = 0.0;
};

// Configuration reaches the core as a plain struct, never through rclcpp.
// Fields are added by the phase that first needs them, so an unused field here
// always means something is unfinished.
struct LocalPlannerConfig
{
  // Two-circle footprint, rear circle at the path point and front circle
  // pushed forward along the heading.
  double collision_circle_radius_m = 0.20;
  double front_collision_circle_offset_m = 0.26;
  double soft_inflation_distance_m = 0.18;
  int occupied_threshold = 50;

  double friction_coeff = 1.0;
  double min_velocity_mps = 0.0;
  double max_velocity_mps = 10.0;
  double max_accel_mps2 = 5.0;
  double max_decel_mps2 = 5.0;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_CORE_TYPES_HPP
