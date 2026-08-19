#ifndef LOCAL_PLANNING_CORE_TYPES_HPP
#define LOCAL_PLANNING_CORE_TYPES_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
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
  double parameter_speed = 0.0;  // |dr/ds| for chord-length station (Newton uses this)
  double normal_x = 0.0;
  double normal_y = 0.0;
  double heading = 0.0;
  double curvature = 0.0;
  double curvature_derivative = 0.0;  // dkappa/ds; piecewise constant on C2 cubic
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

  std::optional<std::size_t> cellAt(const Point & p) const
  {
    if (width <= 0 || height <= 0 || resolution <= 0.0) {
      return std::nullopt;
    }
    const int col = static_cast<int>(std::floor((p.x - origin.x) / resolution));
    const int row = static_cast<int>(std::floor((p.y - origin.y) / resolution));
    if (col < 0 || col >= width || row < 0 || row >= height) {
      return std::nullopt;
    }
    const std::size_t index =
      static_cast<std::size_t>(row) * static_cast<std::size_t>(width) +
      static_cast<std::size_t>(col);
    if (index >= data.size()) {
      return std::nullopt;
    }
    return index;
  }
};

struct VehicleGeometry
{
  double collision_radius_m = 0.20;
  double front_circle_offset_m = 0.26;

  double fullWidthM() const {return 2.0 * collision_radius_m;}
};

struct GridPolicy
{
  int occupied_threshold = 50;
  bool treat_unknown_as_free = true;
  bool treat_out_of_grid_as_free = false;

  bool isOccupied(int8_t value) const
  {
    return value < 0 ? !treat_unknown_as_free : value >= occupied_threshold;
  }
};

// Keep in sync with PlannerDecision.msg intent constants.
enum class PlannerIntent : uint8_t
{
  FOLLOW_RACING_LINE = 0,
  OVERTAKE = 1,
  PASS = 2,
  MERGE = 3
};

std::string intentToString(PlannerIntent intent);

enum class RelativePosition : uint8_t  // longitudinal gap bands (deltaS, includes extents)
{
  NONE = 0,
  BEHIND = 1,
  OVERLAPPING = 2,
  AHEAD_NOT_CLEAR = 3,
  AHEAD_AND_CLEAR = 4
};

std::string relativePositionToString(RelativePosition position);

struct BoundaryState  // G2 anchor: pose + curvature + speed
{
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;    // rad, world frame
  double curvature = 0.0;  // 1/m, positive left
  double speed = 0.0;      // m/s
};

enum class RejectReason : uint8_t  // invalid connections are omitted from the pool
{
  NONE = 0,
  CURVATURE_LIMIT = 1,
  CHART_SINGULAR = 2,
  HEADING_LIMIT = 3
};

struct CurveSample
{
  double s = 0.0;          // arc length from candidate start, m
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;    // rad
  double curvature = 0.0;  // 1/m
  double speed = 0.0;      // m/s; filled by velocity profile
  double raceline_s = std::numeric_limits<double>::quiet_NaN();  // exact station; no re-projection
  double d = 0.0;          // signed lateral offset, positive left
};

} // namespace local_planning

#endif // LOCAL_PLANNING_CORE_TYPES_HPP
