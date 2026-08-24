#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

constexpr double kEps = 1e-9;

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(0.5 * yaw);
  q.z = std::sin(0.5 * yaw);
  q.x = 0.0;
  q.y = 0.0;
  return q;
}

std::vector<std::string> splitSemicolonLine(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ';')) {
    fields.push_back(field);
  }
  return fields;
}

struct Waypoint
{
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct TrackProjection
{
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double lateral_error = 0.0;
  double distance = std::numeric_limits<double>::infinity();
};

struct ScanPoint
{
  double map_x = 0.0;
  double map_y = 0.0;
  double base_x = 0.0;
  double base_y = 0.0;
};

struct ClusterCandidate
{
  std::vector<int> indices;
  double centroid_x = 0.0;
  double centroid_y = 0.0;
  double centroid_base_x = 0.0;
  double centroid_base_y = 0.0;
  double center_x = 0.0;
  double center_y = 0.0;
  double center_s = 0.0;
  double center_yaw = 0.0;
  double projection_distance = std::numeric_limits<double>::infinity();
  double range = 0.0;
  double extent_t = 0.0;
  double extent_n = 0.0;
  double correction_norm = 0.0;
  double score = std::numeric_limits<double>::infinity();
};

}  // namespace

class OpponentLidarDetectorNode : public rclcpp::Node
{
public:
  OpponentLidarDetectorNode()
  : Node("opponent_lidar_detector_node")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    ego_odom_topic_ = declare_parameter<std::string>("ego_odom_topic", "/ego_racecar/odom");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    detected_odom_topic_ = declare_parameter<std::string>("detected_odom_topic", "/opponent/detection_odom");
    detector_marker_topic_ =
      declare_parameter<std::string>("detector_marker_topic", "/opponent/detection_markers");
    detector_debug_topic_ =
      declare_parameter<std::string>("detector_debug_topic", "/opponent/detection_debug");

    waypoint_path_ = declare_parameter<std::string>("waypoint_path", "waypoints/lev_testing/lev_blocked.csv");
    waypoint_path_absolute_ = declare_parameter<bool>("waypoint_path_absolute", false);
    reverse_waypoints_ = declare_parameter<bool>("reverse_waypoints", false);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "ego_racecar/base_link");

    refreshLiveParams();
    loadWaypoints(resolveWaypointPath(waypoint_path_, waypoint_path_absolute_));

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto debug_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, sensor_qos,
      std::bind(&OpponentLidarDetectorNode::scanCallback, this, std::placeholders::_1));
    ego_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ego_odom_topic_, sensor_qos,
      std::bind(&OpponentLidarDetectorNode::egoOdomCallback, this, std::placeholders::_1));
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&OpponentLidarDetectorNode::mapCallback, this, std::placeholders::_1));

    detection_pub_ = create_publisher<nav_msgs::msg::Odometry>(detected_odom_topic_, sensor_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(detector_marker_topic_, debug_qos);
    debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(detector_debug_topic_, debug_qos);

    // Live params are refreshed on a slow timer instead of on every scan
    // (15 Hz x 25 mutex-protected param fetches was real overhead). The
    // refresh also rebuilds the inflated map if inflation/threshold/treat-
    // unknown changed, so live retuning still works.
    params_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&OpponentLidarDetectorNode::onParamsTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "Opponent LiDAR detector ready: scan=%s ego=%s map=%s waypoints=%zu",
      scan_topic_.c_str(), ego_odom_topic_.c_str(), map_topic_.c_str(), waypoints_.size());
  }

  void onParamsTimer()
  {
    refreshLiveParams();
    // Rebuild the inflated mask iff a relevant param changed since last build.
    const int desired_inflation = static_map_.info.resolution > 0.0 ?
      static_cast<int>(std::ceil(wall_inflation_radius_ / static_map_.info.resolution)) :
      0;
    if (
      map_received_ &&
      (inflated_map_.empty() ||
       desired_inflation != inflated_inflation_cells_ ||
       occupied_threshold_ != inflated_occupied_threshold_ ||
       treat_unknown_as_static_ != inflated_treat_unknown_))
    {
      rebuildInflatedMap();
    }
  }

private:
  void refreshLiveParams()
  {
    laser_x_offset_ = declareOrGetDouble("laser_x_offset", 0.0);
    laser_y_offset_ = declareOrGetDouble("laser_y_offset", 0.0);
    laser_yaw_offset_ = declareOrGetDouble("laser_yaw_offset", 0.0);
    opponent_length_ = std::max(0.05, declareOrGetDouble("opponent_length", 0.58));
    opponent_width_ = std::max(0.05, declareOrGetDouble("opponent_width", 0.31));
    max_center_correction_ = std::max(0.0, declareOrGetDouble("max_center_correction", 0.35));

    min_range_ = std::max(0.0, declareOrGetDouble("min_range", 0.15));
    max_range_ = std::max(min_range_, declareOrGetDouble("max_range", 8.0));
    min_detection_range_ = std::max(0.0, declareOrGetDouble("min_detection_range", 0.35));
    max_detection_range_ = std::max(min_detection_range_, declareOrGetDouble("max_detection_range", 6.0));
    front_fov_only_ = declareOrGetBool("front_fov_only", false);
    min_base_x_ = declareOrGetDouble("min_base_x", -0.5);

    require_static_map_ = declareOrGetBool("require_static_map", true);
    occupied_threshold_ = std::clamp(static_cast<int>(declareOrGetInt("occupied_threshold", 50)), 0, 100);
    wall_inflation_radius_ = std::max(0.0, declareOrGetDouble("wall_inflation_radius", 0.18));
    treat_unknown_as_static_ = declareOrGetBool("treat_unknown_as_static", true);

    cluster_tolerance_ = std::max(0.01, declareOrGetDouble("cluster_tolerance", 0.18));
    min_cluster_points_ = std::max(1, static_cast<int>(declareOrGetInt("min_cluster_points", 4)));
    max_cluster_points_ = std::max(min_cluster_points_, static_cast<int>(declareOrGetInt("max_cluster_points", 80)));
    min_cluster_extent_ = std::max(0.0, declareOrGetDouble("min_cluster_extent", 0.08));
    max_cluster_extent_ = std::max(min_cluster_extent_, declareOrGetDouble("max_cluster_extent", 0.9));

    max_raceline_projection_dist_ = std::max(0.0, declareOrGetDouble("max_raceline_projection_dist", 1.2));
    max_candidate_jump_ = std::max(0.0, declareOrGetDouble("max_candidate_jump", 1.5));
    continuity_gate_timeout_ = std::max(0.0, declareOrGetDouble("continuity_gate_timeout", 0.4));
    continuity_weight_ = std::max(0.0, declareOrGetDouble("continuity_weight", 2.0));
    projection_weight_ = std::max(0.0, declareOrGetDouble("projection_weight", 1.0));
    range_weight_ = std::max(0.0, declareOrGetDouble("range_weight", 0.2));
    max_detection_speed_ = std::max(0.1, declareOrGetDouble("max_detection_speed", 12.0));
    detection_publish_rate_hz_ = std::max(1.0, declareOrGetDouble("detection_publish_rate_hz", 20.0));
    // Process every Nth scan only. N=1 means every scan; N=2 halves the
    // detector's CPU + memory-bandwidth footprint (effective ~7.5 Hz at a
    // 15 Hz LiDAR), letting SICK driver keep up with its network buffer
    // when sharing cores with us. Set higher if SICK PLL desyncs persist.
    scan_throttle_n_ = std::max(1, static_cast<int>(declareOrGetInt("scan_throttle_n", 2)));
  }

  double declareOrGetDouble(const std::string & name, double default_value)
  {
    if (!has_parameter(name)) {
      return declare_parameter<double>(name, default_value);
    }
    return get_parameter(name).as_double();
  }

  int64_t declareOrGetInt(const std::string & name, int64_t default_value)
  {
    if (!has_parameter(name)) {
      return declare_parameter<int64_t>(name, default_value);
    }
    return get_parameter(name).as_int();
  }

  bool declareOrGetBool(const std::string & name, bool default_value)
  {
    if (!has_parameter(name)) {
      return declare_parameter<bool>(name, default_value);
    }
    return get_parameter(name).as_bool();
  }

  std::string resolveWaypointPath(const std::string & path, bool absolute)
  {
    if (path.empty() || absolute || (!path.empty() && path.front() == '/')) {
      return path;
    }
    const auto share_dir = ament_index_cpp::get_package_share_directory("mppi_bringup");
    return share_dir + "/" + path;
  }

  void loadWaypoints(const std::string & path)
  {
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "No waypoint_path supplied; detector will not use raceline scoring.");
      return;
    }

    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open detector waypoint_path: %s", path.c_str());
      return;
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      const auto fields = splitSemicolonLine(line);
      if (fields.size() < 4) {
        continue;
      }
      try {
        Waypoint wp;
        wp.s = std::stod(fields[0]);
        wp.x = std::stod(fields[1]);
        wp.y = std::stod(fields[2]);
        wp.yaw = std::stod(fields[3]);
        waypoints_.push_back(wp);
      } catch (const std::exception &) {
        continue;
      }
    }

    if (waypoints_.size() < 2) {
      RCLCPP_ERROR(get_logger(), "Waypoint file has fewer than two usable points: %s", path.c_str());
      waypoints_.clear();
      return;
    }

    if (reverse_waypoints_) {
      reverseWaypoints();
      RCLCPP_INFO(get_logger(), "reverse_waypoints=true: centerline flipped to match opposite travel direction.");
    }

    const auto & first = waypoints_.front();
    const auto & last = waypoints_.back();
    track_length_ = last.s + std::hypot(first.x - last.x, first.y - last.y);
    if (track_length_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid track length from waypoint_path: %s", path.c_str());
      waypoints_.clear();
      track_length_ = 0.0;
    }
  }

  // Reverse the loaded centerline so its arc-length s increases *with* travel
  // when racing the track in the opposite direction, keeping this node's
  // progress frame aligned with the predictor's. The raceline yaw used
  // downstream (gating, center correction) is recomputed from segment direction
  // in projectToTrack, so the stored yaw needs no flip here. Applied once at
  // load time; not a live param.
  void reverseWaypoints()
  {
    std::reverse(waypoints_.begin(), waypoints_.end());
    waypoints_.front().s = 0.0;
    for (std::size_t i = 1; i < waypoints_.size(); ++i) {
      const double ds = std::hypot(
        waypoints_[i].x - waypoints_[i - 1].x,
        waypoints_[i].y - waypoints_[i - 1].y);
      waypoints_[i].s = waypoints_[i - 1].s + ds;
    }
  }

  double normalizeS(double s) const
  {
    if (track_length_ <= 0.0) {
      return s;
    }
    s = std::fmod(s, track_length_);
    if (s < 0.0) {
      s += track_length_;
    }
    return s;
  }

  double unwrapS(double measured_s, double reference_s) const
  {
    if (track_length_ <= 0.0) {
      return measured_s;
    }
    double unwrapped = measured_s;
    while (unwrapped - reference_s > 0.5 * track_length_) {
      unwrapped -= track_length_;
    }
    while (unwrapped - reference_s < -0.5 * track_length_) {
      unwrapped += track_length_;
    }
    return unwrapped;
  }

  TrackProjection projectToTrack(double x, double y) const
  {
    TrackProjection best;
    if (waypoints_.size() < 2) {
      best.x = x;
      best.y = y;
      best.yaw = ego_yaw_;
      best.distance = 0.0;
      return best;
    }

    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      const Waypoint & a = waypoints_[i];
      const Waypoint & b = (i + 1 < waypoints_.size()) ? waypoints_[i + 1] : waypoints_.front();
      const double b_s = (i + 1 < waypoints_.size()) ? b.s : track_length_;
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double len2 = std::max(kEps, dx * dx + dy * dy);
      const double t = std::clamp(((x - a.x) * dx + (y - a.y) * dy) / len2, 0.0, 1.0);
      const double px = a.x + t * dx;
      const double py = a.y + t * dy;
      const double ex = x - px;
      const double ey = y - py;
      const double dist = std::hypot(ex, ey);

      if (dist < best.distance) {
        best.distance = dist;
        best.s = normalizeS(a.s + t * (b_s - a.s));
        best.x = px;
        best.y = py;
        best.yaw = std::atan2(dy, dx);
        const double nx = -std::sin(best.yaw);
        const double ny = std::cos(best.yaw);
        best.lateral_error = ex * nx + ey * ny;
      }
    }
    return best;
  }

  // O(1) point-vs-inflated-map test. The inflation is precomputed once on
  // map receipt (or on relevant param change) by rebuildInflatedMap(); the
  // old implementation did a (2*inflation+1)^2 cell sweep PER scan point per
  // scan, which dominated the scan callback budget on Jetson and contended
  // with JAX's GPU<->host DMA via memory bandwidth.
  bool pointNearStaticMap(double x, double y) const
  {
    if (!map_received_ || inflated_map_.empty()) {
      return require_static_map_;
    }
    const auto & info = static_map_.info;
    const int mx = static_cast<int>((x - info.origin.position.x) / info.resolution);
    const int my = static_cast<int>((y - info.origin.position.y) / info.resolution);
    if (mx < 0 || mx >= static_cast<int>(info.width) || my < 0 || my >= static_cast<int>(info.height)) {
      return true;  // out-of-bounds matches old semantics
    }
    return inflated_map_[static_cast<std::size_t>(my) * info.width + mx] != 0;
  }

  // Brute-force precompute of the dilated occupancy mask. Runs once per map
  // receipt and again only if inflation/threshold/treat-unknown params change
  // (detected in onParamsTimer). One-time cost ~10-30 ms on a Levine-sized
  // map (~340x255). Per-scan cost afterwards is O(1) per point.
  void rebuildInflatedMap()
  {
    if (!map_received_) {
      inflated_map_.clear();
      return;
    }
    const auto & info = static_map_.info;
    if (info.resolution <= 0.0 || info.width == 0 || info.height == 0) {
      inflated_map_.clear();
      return;
    }
    const int width = static_cast<int>(info.width);
    const int height = static_cast<int>(info.height);
    const int inflation_cells = std::max(0,
      static_cast<int>(std::ceil(wall_inflation_radius_ / info.resolution)));

    // Step 1: blocked-base mask (occupied OR unknown-as-static).
    std::vector<uint8_t> blocked(static_cast<std::size_t>(width) * height, 0);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int8_t v = static_map_.data[y * width + x];
        if (v >= occupied_threshold_ || (treat_unknown_as_static_ && v < 0)) {
          blocked[static_cast<std::size_t>(y) * width + x] = 1;
        }
      }
    }

    // Step 2: dilation. For each blocked cell, mark a (2k+1)^2 window in the
    // result. We iterate over BLOCKED source cells (typically a small
    // fraction of the map) instead of every cell, which is ~10x faster than
    // the naive "for each cell, check k-window of source".
    inflated_map_.assign(static_cast<std::size_t>(width) * height, 0);
    if (inflation_cells == 0) {
      inflated_map_ = blocked;
    } else {
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          if (!blocked[static_cast<std::size_t>(y) * width + x]) {
            continue;
          }
          const int y_lo = std::max(0, y - inflation_cells);
          const int y_hi = std::min(height - 1, y + inflation_cells);
          const int x_lo = std::max(0, x - inflation_cells);
          const int x_hi = std::min(width - 1, x + inflation_cells);
          for (int yy = y_lo; yy <= y_hi; ++yy) {
            std::fill(
              inflated_map_.begin() + static_cast<std::size_t>(yy) * width + x_lo,
              inflated_map_.begin() + static_cast<std::size_t>(yy) * width + x_hi + 1,
              uint8_t{1});
          }
        }
      }
    }

    inflated_inflation_cells_ = inflation_cells;
    inflated_occupied_threshold_ = occupied_threshold_;
    inflated_treat_unknown_ = treat_unknown_as_static_;
    RCLCPP_INFO(
      get_logger(),
      "Inflated static-map mask rebuilt: %dx%d, inflation_cells=%d",
      width, height, inflation_cells);
  }

  // Grid-bucketed connected-components. Cell size = cluster_tolerance_, so
  // any point within tolerance of `idx` lives in idx's cell or one of its 8
  // neighbours. Replaces O(N^2) brute-force region-grow with O(N) expected.
  // Functional output is identical (BFS over the same neighbour graph) — only
  // the candidate-pair generation changes.
  std::vector<std::vector<int>> clusterPoints(const std::vector<ScanPoint> & points) const
  {
    std::vector<std::vector<int>> clusters;
    if (points.empty()) {
      return clusters;
    }

    const double cell = std::max(cluster_tolerance_, 1e-3);
    const double inv_cell = 1.0 / cell;
    const double tol2 = cluster_tolerance_ * cluster_tolerance_;

    auto key_of = [inv_cell](double x, double y) -> int64_t {
      // 32-bit halves -> 64-bit key. Range ~ +/-1e9 cells which is safely
      // beyond any racetrack scale.
      const int32_t cx = static_cast<int32_t>(std::floor(x * inv_cell));
      const int32_t cy = static_cast<int32_t>(std::floor(y * inv_cell));
      return (static_cast<int64_t>(cx) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(cy));
    };

    std::unordered_map<int64_t, std::vector<int>> grid;
    grid.reserve(points.size() * 2);
    std::vector<int32_t> cx_of(points.size());
    std::vector<int32_t> cy_of(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      const int32_t cx = static_cast<int32_t>(std::floor(points[i].map_x * inv_cell));
      const int32_t cy = static_cast<int32_t>(std::floor(points[i].map_y * inv_cell));
      cx_of[i] = cx;
      cy_of[i] = cy;
      const int64_t k = (static_cast<int64_t>(cx) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(cy));
      grid[k].push_back(static_cast<int>(i));
    }

    std::vector<bool> visited(points.size(), false);
    std::queue<int> q;
    for (std::size_t seed = 0; seed < points.size(); ++seed) {
      if (visited[seed]) {
        continue;
      }
      visited[seed] = true;
      q.push(static_cast<int>(seed));
      std::vector<int> cluster;
      while (!q.empty()) {
        const int idx = q.front();
        q.pop();
        cluster.push_back(idx);
        const int32_t cx = cx_of[idx];
        const int32_t cy = cy_of[idx];
        for (int32_t dy = -1; dy <= 1; ++dy) {
          for (int32_t dx = -1; dx <= 1; ++dx) {
            const int64_t nk = (static_cast<int64_t>(cx + dx) << 32) ^
              static_cast<int64_t>(static_cast<uint32_t>(cy + dy));
            auto it = grid.find(nk);
            if (it == grid.end()) {
              continue;
            }
            for (const int j : it->second) {
              if (visited[j]) {
                continue;
              }
              const double ddx = points[idx].map_x - points[j].map_x;
              const double ddy = points[idx].map_y - points[j].map_y;
              if (ddx * ddx + ddy * ddy <= tol2) {
                visited[j] = true;
                q.push(j);
              }
            }
          }
        }
      }
      clusters.push_back(std::move(cluster));
    }
    return clusters;
  }

  // Per-scan rejection tally so the user can see *why* clusters fail.
  // Filled by buildCandidate, aggregated in selectCandidate, summarized in
  // scanCallback (throttled). Not load-bearing — pure diagnostic.
  struct RejectTally {
    int accepted = 0;
    int cluster_size = 0;
    int base_x = 0;
    int proj_distance = 0;
    int extent = 0;
    int proj_distance_post = 0;
    int implied_speed = 0;
    int candidate_jump = 0;
    double best_proj_distance = std::numeric_limits<double>::infinity();
  };

  enum class CandidateOutcome {
    kAccepted, kClusterSize, kBaseX, kProjDistance, kExtent,
    kProjDistancePost, kImpliedSpeed, kCandidateJump,
  };

  CandidateOutcome buildCandidate(
    const std::vector<int> & cluster,
    const std::vector<ScanPoint> & points,
    const rclcpp::Time & stamp,
    ClusterCandidate & candidate) const
  {
    if (
      cluster.size() < static_cast<std::size_t>(min_cluster_points_) ||
      cluster.size() > static_cast<std::size_t>(max_cluster_points_))
    {
      return CandidateOutcome::kClusterSize;
    }

    for (const int idx : cluster) {
      candidate.centroid_x += points[idx].map_x;
      candidate.centroid_y += points[idx].map_y;
      candidate.centroid_base_x += points[idx].base_x;
      candidate.centroid_base_y += points[idx].base_y;
    }
    const double inv_n = 1.0 / static_cast<double>(cluster.size());
    candidate.centroid_x *= inv_n;
    candidate.centroid_y *= inv_n;
    candidate.centroid_base_x *= inv_n;
    candidate.centroid_base_y *= inv_n;
    candidate.indices = cluster;
    candidate.range = std::hypot(candidate.centroid_base_x, candidate.centroid_base_y);

    if (candidate.centroid_base_x < min_base_x_) {
      return CandidateOutcome::kBaseX;
    }

    const TrackProjection proj = projectToTrack(candidate.centroid_x, candidate.centroid_y);
    if (proj.distance > max_raceline_projection_dist_) {
      candidate.projection_distance = proj.distance;  // for tally summary
      return CandidateOutcome::kProjDistance;
    }

    const double tx = std::cos(proj.yaw);
    const double ty = std::sin(proj.yaw);
    const double nx = -std::sin(proj.yaw);
    const double ny = std::cos(proj.yaw);

    double min_t = std::numeric_limits<double>::infinity();
    double max_t = -std::numeric_limits<double>::infinity();
    double min_n = std::numeric_limits<double>::infinity();
    double max_n = -std::numeric_limits<double>::infinity();
    for (const int idx : cluster) {
      const double dx = points[idx].map_x - candidate.centroid_x;
      const double dy = points[idx].map_y - candidate.centroid_y;
      const double local_t = dx * tx + dy * ty;
      const double local_n = dx * nx + dy * ny;
      min_t = std::min(min_t, local_t);
      max_t = std::max(max_t, local_t);
      min_n = std::min(min_n, local_n);
      max_n = std::max(max_n, local_n);
    }
    candidate.extent_t = std::max(0.0, max_t - min_t);
    candidate.extent_n = std::max(0.0, max_n - min_n);
    const double max_extent = std::max(candidate.extent_t, candidate.extent_n);
    if (max_extent < min_cluster_extent_ || max_extent > max_cluster_extent_) {
      return CandidateOutcome::kExtent;
    }

    const double view_x = candidate.centroid_x - ego_x_;
    const double view_y = candidate.centroid_y - ego_y_;
    const double view_norm = std::max(kEps, std::hypot(view_x, view_y));
    const double ux = view_x / view_norm;
    const double uy = view_y / view_norm;
    const double dir_t = ux * tx + uy * ty;
    const double dir_n = ux * nx + uy * ny;
    const double missing_t = std::max(0.0, 0.5 * opponent_length_ - 0.5 * candidate.extent_t);
    const double missing_n = std::max(0.0, 0.5 * opponent_width_ - 0.5 * candidate.extent_n);
    double corr_x = tx * dir_t * missing_t + nx * dir_n * missing_n;
    double corr_y = ty * dir_t * missing_t + ny * dir_n * missing_n;
    candidate.correction_norm = std::hypot(corr_x, corr_y);
    if (candidate.correction_norm > max_center_correction_ && candidate.correction_norm > kEps) {
      const double scale = max_center_correction_ / candidate.correction_norm;
      corr_x *= scale;
      corr_y *= scale;
      candidate.correction_norm = max_center_correction_;
    }

    candidate.center_x = candidate.centroid_x + corr_x;
    candidate.center_y = candidate.centroid_y + corr_y;
    const TrackProjection corrected_proj = projectToTrack(candidate.center_x, candidate.center_y);
    candidate.center_s = corrected_proj.s;
    candidate.center_yaw = corrected_proj.yaw;
    candidate.projection_distance = corrected_proj.distance;

    if (candidate.projection_distance > max_raceline_projection_dist_) {
      return CandidateOutcome::kProjDistancePost;
    }

    candidate.score =
      projection_weight_ * candidate.projection_distance +
      range_weight_ * candidate.range;

    if (has_last_detection_) {
      const double age = (stamp - last_detection_stamp_).seconds();
      const double jump = std::hypot(
        candidate.center_x - last_detection_x_,
        candidate.center_y - last_detection_y_);
      if (age >= 1e-3 && age < continuity_gate_timeout_) {
        const double candidate_s = unwrapS(candidate.center_s, last_detection_s_);
        const double implied_progress_speed = std::abs(candidate_s - last_detection_s_) / age;
        if (implied_progress_speed > max_detection_speed_) {
          return CandidateOutcome::kImpliedSpeed;
        }
      }
      if (age >= 0.0 && age < continuity_gate_timeout_ && jump > max_candidate_jump_) {
        return CandidateOutcome::kCandidateJump;
      }
      if (age >= 0.0 && age < continuity_gate_timeout_) {
        candidate.score += continuity_weight_ * jump;
      }
    }

    return CandidateOutcome::kAccepted;
  }

  bool selectCandidate(
    const std::vector<ScanPoint> & points,
    const std::vector<std::vector<int>> & clusters,
    const rclcpp::Time & stamp,
    ClusterCandidate & selected,
    RejectTally & tally) const
  {
    bool found = false;
    for (const auto & cluster : clusters) {
      ClusterCandidate candidate;
      const auto outcome = buildCandidate(cluster, points, stamp, candidate);
      // Track the best (smallest) projection distance we've seen this scan
      // — tells the user how close they are to the threshold.
      if (std::isfinite(candidate.projection_distance)) {
        tally.best_proj_distance = std::min(
          tally.best_proj_distance, candidate.projection_distance);
      }
      switch (outcome) {
        case CandidateOutcome::kAccepted: tally.accepted++; break;
        case CandidateOutcome::kClusterSize: tally.cluster_size++; continue;
        case CandidateOutcome::kBaseX: tally.base_x++; continue;
        case CandidateOutcome::kProjDistance: tally.proj_distance++; continue;
        case CandidateOutcome::kExtent: tally.extent++; continue;
        case CandidateOutcome::kProjDistancePost: tally.proj_distance_post++; continue;
        case CandidateOutcome::kImpliedSpeed: tally.implied_speed++; continue;
        case CandidateOutcome::kCandidateJump: tally.candidate_jump++; continue;
      }
      if (!found || candidate.score < selected.score) {
        selected = candidate;
        found = true;
      }
    }
    return found;
  }

  bool shouldPublishDetection(const rclcpp::Time & stamp)
  {
    if (!has_last_detection_publish_) {
      return true;
    }
    const double min_dt = 1.0 / detection_publish_rate_hz_;
    return (stamp - last_detection_publish_stamp_).seconds() >= min_dt;
  }

  void publishDetection(const ClusterCandidate & candidate, const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = "opponent_lidar_detection";
    odom.pose.pose.position.x = candidate.center_x;
    odom.pose.pose.position.y = candidate.center_y;
    odom.pose.pose.orientation = yawToQuaternion(candidate.center_yaw);

    double speed = 0.0;
    if (has_last_detection_) {
      const double dt = (stamp - last_detection_stamp_).seconds();
      if (dt > 1e-3) {
        const double unwrapped_s = unwrapS(candidate.center_s, last_detection_s_);
        speed = (unwrapped_s - last_detection_s_) / dt;
        speed = std::clamp(speed, 0.0, max_detection_speed_);
      }
    }
    odom.twist.twist.linear.x = speed;
    detection_pub_->publish(odom);

    last_detection_x_ = candidate.center_x;
    last_detection_y_ = candidate.center_y;
    last_detection_s_ = has_last_detection_ ? unwrapS(candidate.center_s, last_detection_s_) : candidate.center_s;
    last_detection_stamp_ = stamp;
    has_last_detection_ = true;
    last_detection_publish_stamp_ = stamp;
    has_last_detection_publish_ = true;
  }

  void publishMarkers(
    const std::vector<ScanPoint> & dynamic_points,
    const ClusterCandidate * selected,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    visualization_msgs::msg::Marker dyn;
    dyn.header.stamp = stamp;
    dyn.header.frame_id = frame_id_;
    dyn.ns = "opponent_lidar_detection";
    dyn.id = 0;
    dyn.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    dyn.action = visualization_msgs::msg::Marker::ADD;
    dyn.scale.x = 0.06;
    dyn.scale.y = 0.06;
    dyn.scale.z = 0.06;
    dyn.color.a = 0.75;
    dyn.color.r = 1.0;
    dyn.color.g = 0.1;
    dyn.color.b = 0.1;
    for (const auto & p : dynamic_points) {
      geometry_msgs::msg::Point point;
      point.x = p.map_x;
      point.y = p.map_y;
      point.z = 0.08;
      dyn.points.push_back(point);
    }
    markers.markers.push_back(dyn);

    if (selected != nullptr) {
      visualization_msgs::msg::Marker center;
      center.header.stamp = stamp;
      center.header.frame_id = frame_id_;
      center.ns = "opponent_lidar_detection";
      center.id = 1;
      center.type = visualization_msgs::msg::Marker::ARROW;
      center.action = visualization_msgs::msg::Marker::ADD;
      center.pose.position.x = selected->center_x;
      center.pose.position.y = selected->center_y;
      center.pose.position.z = 0.2;
      center.pose.orientation = yawToQuaternion(selected->center_yaw);
      center.scale.x = opponent_length_;
      center.scale.y = opponent_width_;
      center.scale.z = 0.12;
      center.color.a = 0.9;
      center.color.r = 0.1;
      center.color.g = 1.0;
      center.color.b = 0.25;
      markers.markers.push_back(center);

      visualization_msgs::msg::Marker text;
      text.header.stamp = stamp;
      text.header.frame_id = frame_id_;
      text.ns = "opponent_lidar_detection";
      text.id = 2;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = selected->center_x;
      text.pose.position.y = selected->center_y;
      text.pose.position.z = 0.65;
      text.scale.z = 0.2;
      text.color.a = 0.95;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      std::ostringstream ss;
      ss << "det score=" << std::fixed << std::setprecision(2) << selected->score
         << " d=" << selected->projection_distance;
      text.text = ss.str();
      markers.markers.push_back(text);
    }

    marker_pub_->publish(markers);
  }

  void publishDebug(
    std::size_t dynamic_count,
    std::size_t cluster_count,
    const ClusterCandidate * selected,
    double progress_speed) const
  {
    std_msgs::msg::Float32MultiArray debug;
    debug.data = {
      static_cast<float>(dynamic_count),
      static_cast<float>(cluster_count),
      selected ? static_cast<float>(selected->score) : -1.0f,
      selected ? static_cast<float>(selected->range) : -1.0f,
      selected ? static_cast<float>(selected->projection_distance) : -1.0f,
      selected ? static_cast<float>(selected->correction_norm) : -1.0f,
      selected ? static_cast<float>(selected->extent_t) : -1.0f,
      selected ? static_cast<float>(selected->extent_n) : -1.0f,
      static_cast<float>(progress_speed),
      map_received_ ? 1.0f : 0.0f
    };
    debug_pub_->publish(debug);
  }

  void egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double yaw = yawFromQuaternion(msg->pose.pose.orientation);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      have_ego_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Ignoring non-finite ego odom measurement.");
      return;
    }

    ego_x_ = x;
    ego_y_ = y;
    ego_yaw_ = yaw;
    ego_stamp_ = msg->header.stamp;
    have_ego_ = true;
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    static_map_ = *msg;
    map_received_ = true;
    rebuildInflatedMap();
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Throttle: only process every scan_throttle_n_-th scan. Drops the
    // detector's per-scan CPU + memory-bandwidth footprint proportionally,
    // which lets SICK driver keep up on shared cores (otherwise SICK's
    // PLL desyncs and we get bad scan timestamps). Default 2 = 7.5 Hz
    // detection rate at a 15 Hz LiDAR — still plenty for tracking opp.
    if (++scan_skip_counter_ % scan_throttle_n_ != 0) {
      return;
    }

    // refreshLiveParams() moved to onParamsTimer (2 Hz) — see ctor.
    const auto cb_t0 = std::chrono::steady_clock::now();

    if (!have_ego_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for ego odom.");
      return;
    }
    if (require_static_map_ && !map_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for static map.");
      return;
    }

    const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
      now() : rclcpp::Time(msg->header.stamp);

    const double cy = std::cos(ego_yaw_);
    const double sy = std::sin(ego_yaw_);
    const double cl = std::cos(laser_yaw_offset_);
    const double sl = std::sin(laser_yaw_offset_);
    std::vector<ScanPoint> dynamic_points;
    dynamic_points.reserve(msg->ranges.size());

    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const double r = msg->ranges[i];
      if (!std::isfinite(r) || r < min_range_ || r > max_range_ || r > msg->range_max) {
        continue;
      }
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      if (front_fov_only_ && std::abs(angle) > 0.5 * M_PI) {
        continue;
      }

      const double lx = r * std::cos(angle);
      const double ly = r * std::sin(angle);
      const double bx = laser_x_offset_ + cl * lx - sl * ly;
      const double by = laser_y_offset_ + sl * lx + cl * ly;
      const double base_range = std::hypot(bx, by);
      if (base_range < min_detection_range_ || base_range > max_detection_range_ || bx < min_base_x_) {
        continue;
      }

      ScanPoint point;
      point.base_x = bx;
      point.base_y = by;
      point.map_x = ego_x_ + cy * bx - sy * by;
      point.map_y = ego_y_ + sy * bx + cy * by;
      if (pointNearStaticMap(point.map_x, point.map_y)) {
        continue;
      }
      dynamic_points.push_back(point);
    }

    const auto clusters = clusterPoints(dynamic_points);
    ClusterCandidate selected;
    RejectTally tally;
    const bool found = selectCandidate(dynamic_points, clusters, stamp, selected, tally);
    double progress_speed = 0.0;
    if (found && has_last_detection_) {
      const double dt = (stamp - last_detection_stamp_).seconds();
      if (dt > 1e-3) {
        progress_speed = std::clamp(
          (unwrapS(selected.center_s, last_detection_s_) - last_detection_s_) / dt,
          0.0,
          max_detection_speed_);
      }
    }

    if (found) {
      if (shouldPublishDetection(stamp)) {
        publishDetection(selected, stamp);
      }
      publishMarkers(dynamic_points, &selected, stamp);
      publishDebug(dynamic_points.size(), clusters.size(), &selected, progress_speed);
    } else {
      publishMarkers(dynamic_points, nullptr, stamp);
      publishDebug(dynamic_points.size(), clusters.size(), nullptr, 0.0);
    }

    const auto cb_dt_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - cb_t0).count();
    const double best_d = std::isfinite(tally.best_proj_distance) ?
      tally.best_proj_distance : -1.0;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "scanCallback %.2f ms | dyn_pts=%zu clusters=%zu | "
      "accepted=%d reject{size=%d, base_x=%d, proj=%d, extent=%d, "
      "proj_post=%d, speed=%d, jump=%d} best_proj_d=%.2f (thresh=%.2f)",
      cb_dt_ms, dynamic_points.size(), clusters.size(),
      tally.accepted, tally.cluster_size, tally.base_x, tally.proj_distance,
      tally.extent, tally.proj_distance_post, tally.implied_speed,
      tally.candidate_jump, best_d, max_raceline_projection_dist_);
  }

  std::string scan_topic_;
  std::string ego_odom_topic_;
  std::string map_topic_;
  std::string detected_odom_topic_;
  std::string detector_marker_topic_;
  std::string detector_debug_topic_;
  std::string waypoint_path_;
  bool waypoint_path_absolute_ = false;
  bool reverse_waypoints_ = false;
  std::string frame_id_ = "map";
  std::string base_frame_id_ = "ego_racecar/base_link";

  double laser_x_offset_ = 0.0;
  double laser_y_offset_ = 0.0;
  double laser_yaw_offset_ = 0.0;
  double opponent_length_ = 0.58;
  double opponent_width_ = 0.31;
  double max_center_correction_ = 0.35;
  double min_range_ = 0.15;
  double max_range_ = 8.0;
  double min_detection_range_ = 0.35;
  double max_detection_range_ = 6.0;
  bool front_fov_only_ = false;
  double min_base_x_ = -0.5;
  bool require_static_map_ = true;
  int occupied_threshold_ = 50;
  double wall_inflation_radius_ = 0.18;
  bool treat_unknown_as_static_ = true;
  double cluster_tolerance_ = 0.18;
  int min_cluster_points_ = 4;
  int max_cluster_points_ = 80;
  double min_cluster_extent_ = 0.08;
  double max_cluster_extent_ = 0.9;
  double max_raceline_projection_dist_ = 1.2;
  double max_candidate_jump_ = 1.5;
  double continuity_gate_timeout_ = 0.4;
  double continuity_weight_ = 2.0;
  double projection_weight_ = 1.0;
  double range_weight_ = 0.2;
  double max_detection_speed_ = 12.0;
  double detection_publish_rate_hz_ = 20.0;
  int scan_throttle_n_ = 1;
  int scan_skip_counter_ = 0;

  bool have_ego_ = false;
  double ego_x_ = 0.0;
  double ego_y_ = 0.0;
  double ego_yaw_ = 0.0;
  rclcpp::Time ego_stamp_;

  bool map_received_ = false;
  nav_msgs::msg::OccupancyGrid static_map_;
  // Precomputed inflated occupancy mask (1 = blocked-after-inflation), same
  // dimensions as static_map_. Rebuilt from rebuildInflatedMap().
  std::vector<uint8_t> inflated_map_;
  int inflated_inflation_cells_ = -1;
  int inflated_occupied_threshold_ = -1;
  bool inflated_treat_unknown_ = true;
  rclcpp::TimerBase::SharedPtr params_timer_;

  std::vector<Waypoint> waypoints_;
  double track_length_ = 0.0;

  bool has_last_detection_ = false;
  double last_detection_x_ = 0.0;
  double last_detection_y_ = 0.0;
  double last_detection_s_ = 0.0;
  rclcpp::Time last_detection_stamp_;
  bool has_last_detection_publish_ = false;
  rclcpp::Time last_detection_publish_stamp_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr detection_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpponentLidarDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
