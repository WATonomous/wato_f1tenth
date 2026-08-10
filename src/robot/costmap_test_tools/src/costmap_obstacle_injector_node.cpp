// Stamps synthetic obstacles into the raw costmap so the local planner can be
// exercised in sim without a second car.  Sits between costmap_node and the
// occupancy grid frame adapter:
//
//   /scan -> costmap_node -> /costmap -> [this] -> /costmap_injected -> adapter
//
// Injecting before the adapter is what makes the opponent detector and the
// collision checker see byte-identical cells.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "local_planning/core/types.hpp"
#include "local_planning/reference/raceline_reference.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

enum class ObstacleKind
{
  Static,           // holds a fixed map pose, never touched again
  RacelineStatic,   // parked at an (s, d) on the line; respawn moves it to a new s
  RacelineMover,    // advances along the line every cycle
};

ObstacleKind parseKind(const std::string & text)
{
  if (text == "raceline_mover") {
    return ObstacleKind::RacelineMover;
  }
  if (text == "raceline_static") {
    return ObstacleKind::RacelineStatic;
  }
  return ObstacleKind::Static;
}

struct Obstacle
{
  std::string name;
  ObstacleKind kind = ObstacleKind::Static;

  // Frenet state, meaningful for both raceline kinds.
  double s = 0.0;
  double d = 0.0;
  double speed_scale = 0.6;

  // Current pose in the map frame.  Derived from (s, d) for the raceline kinds.
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  double length_m = 0.50;
  double width_m = 0.30;

  // Raceline-anchored obstacles have no valid pose until a raceline arrives.
  bool anchored() const {return kind != ObstacleKind::Static;}
};

}  // namespace

class CostmapObstacleInjector : public rclcpp::Node
{
public:
  CostmapObstacleInjector()
  : Node("costmap_obstacle_injector_node")
  {
    enabled_ = declare_parameter<bool>("enabled", true);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    obstacle_value_ = static_cast<int8_t>(declare_parameter<int>("obstacle_value", 100));
    min_ego_distance_m_ = declare_parameter<double>("min_ego_distance_m", 0.5);
    clicked_obstacle_size_m_ = declare_parameter<double>("clicked_obstacle_size_m", 0.30);
    ego_frame_ = declare_parameter<std::string>("ego_frame", "base_link");
    respawn_min_ego_distance_m_ =
      declare_parameter<double>("respawn_min_ego_distance_m", 4.0);
    respawn_lateral_range_m_ = declare_parameter<double>("respawn_lateral_range_m", 0.0);

    const auto raw_grid_topic = declare_parameter<std::string>("raw_grid_topic", "/costmap");
    const auto injected_grid_topic =
      declare_parameter<std::string>("injected_grid_topic", "/costmap_injected");
    const auto raceline_topic =
      declare_parameter<std::string>("raceline_topic", "/global_planner/path");
    const auto marker_topic =
      declare_parameter<std::string>("marker_topic", "/injected_obstacles_viz");
    const auto respawn_topic =
      declare_parameter<std::string>("respawn_topic", "/inject_obstacle/respawn");

    for (const auto & name :
      declare_parameter<std::vector<std::string>>("obstacle_names", std::vector<std::string>{}))
    {
      Obstacle obstacle;
      obstacle.name = name;
      obstacle.kind = parseKind(declare_parameter<std::string>(name + ".kind", "static"));
      obstacle.s = declare_parameter<double>(name + ".start_s", 0.0);
      obstacle.d = declare_parameter<double>(name + ".d", 0.0);
      obstacle.speed_scale = declare_parameter<double>(name + ".speed_scale", 0.6);
      obstacle.x = declare_parameter<double>(name + ".x", 0.0);
      obstacle.y = declare_parameter<double>(name + ".y", 0.0);
      obstacle.yaw = declare_parameter<double>(name + ".yaw", 0.0);
      obstacle.length_m = declare_parameter<double>(name + ".length_m", 0.50);
      obstacle.width_m = declare_parameter<double>(name + ".width_m", 0.30);
      obstacles_.push_back(obstacle);
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(injected_grid_topic, 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 1);

    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      raw_grid_topic, 1,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {onGrid(*msg);});

    raceline_sub_ = create_subscription<nav_msgs::msg::Path>(
      raceline_topic, rclcpp::QoS(1).transient_local().reliable(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {onRaceline(*msg);});

    // RViz "Publish Point" drops a static block wherever you click.
    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 1,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {onClickedPoint(*msg);});

    // One `ros2 topic pub` teleports every raceline_static block to a fresh
    // random spot on the line, so a scenario can be re-rolled without a relaunch.
    respawn_sub_ = create_subscription<std_msgs::msg::Empty>(
      respawn_topic, 1,
      [this](const std_msgs::msg::Empty::SharedPtr) {onRespawn();});

    RCLCPP_INFO(
      get_logger(), "Injector %s: %s -> %s, %zu configured obstacle(s)",
      enabled_ ? "enabled" : "disabled (passthrough)",
      raw_grid_topic.c_str(), injected_grid_topic.c_str(), obstacles_.size());
  }

private:
  void onRaceline(const nav_msgs::msg::Path & msg)
  {
    std::vector<local_planning::Point> points;
    points.reserve(msg.poses.size());
    for (const auto & pose : msg.poses) {
      points.emplace_back(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }

    if (!reference_.setRacingLine(points)) {
      RCLCPP_WARN(get_logger(), "Raceline rejected (%zu points); movers stay parked",
        points.size());
      return;
    }
    RCLCPP_INFO(get_logger(), "Raceline accepted, %.2f m loop", reference_.totalLength());

    // Anchored obstacles were parked at an (s, d) with no line to resolve it
    // against; now they have one.
    for (auto & obstacle : obstacles_) {
      if (obstacle.anchored()) {
        placeOnRaceline(obstacle);
      }
    }
  }

  // Re-rolls every raceline_static obstacle onto a fresh random s.  Movers and
  // clicked blocks are left alone.
  void onRespawn()
  {
    if (!reference_.valid()) {
      RCLCPP_WARN(get_logger(), "Respawn ignored: no raceline yet");
      return;
    }

    const auto ego = lookupEgoInMap();
    if (!ego) {
      RCLCPP_WARN(
        get_logger(), "Respawn: ego pose unknown, obstacle may land on top of the car");
    }

    std::uniform_real_distribution<double> s_dist(0.0, reference_.totalLength());
    std::uniform_real_distribution<double> d_dist(
      -respawn_lateral_range_m_, respawn_lateral_range_m_);

    int respawned = 0;
    for (auto & obstacle : obstacles_) {
      if (obstacle.kind != ObstacleKind::RacelineStatic) {
        continue;
      }

      // Rejection sample so the new spot is not right on the car's nose.  A
      // bounded number of tries keeps this terminating on a short loop where
      // no candidate clears the radius; the last draw is used regardless.
      for (int attempt = 0; attempt < 32; ++attempt) {
        obstacle.s = s_dist(rng_);
        obstacle.d = d_dist(rng_);
        placeOnRaceline(obstacle);
        if (!ego ||
          std::hypot(obstacle.x - ego->x(), obstacle.y - ego->y()) >=
          respawn_min_ego_distance_m_)
        {
          break;
        }
      }

      ++respawned;
      RCLCPP_INFO(
        get_logger(), "Respawned '%s' at s=%.2f d=%.2f -> (%.2f, %.2f)",
        obstacle.name.c_str(), obstacle.s, obstacle.d, obstacle.x, obstacle.y);
    }

    if (respawned == 0) {
      RCLCPP_WARN(get_logger(), "Respawn: no obstacle of kind 'raceline_static' configured");
    }
  }

  void placeOnRaceline(Obstacle & obstacle)
  {
    const auto sample = reference_.sampleAtS(obstacle.s);
    obstacle.x = sample.x + obstacle.d * sample.normal_x;
    obstacle.y = sample.y + obstacle.d * sample.normal_y;
    obstacle.yaw = sample.heading;
  }

  std::optional<tf2::Vector3> lookupEgoInMap() const
  {
    try {
      const auto transform =
        tf_buffer_->lookupTransform(map_frame_, ego_frame_, tf2::TimePointZero);
      return tf2::Vector3(
        transform.transform.translation.x, transform.transform.translation.y, 0.0);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(), "Cannot transform '%s' -> '%s': %s",
        ego_frame_.c_str(), map_frame_.c_str(), ex.what());
      return std::nullopt;
    }
  }

  void onClickedPoint(const geometry_msgs::msg::PointStamped & msg)
  {
    Obstacle obstacle;
    obstacle.name = "clicked_" + std::to_string(obstacles_.size());
    obstacle.x = msg.point.x;
    obstacle.y = msg.point.y;
    obstacle.length_m = clicked_obstacle_size_m_;
    obstacle.width_m = clicked_obstacle_size_m_;
    obstacles_.push_back(obstacle);
    RCLCPP_INFO(
      get_logger(), "Added %s at (%.2f, %.2f) in %s",
      obstacle.name.c_str(), obstacle.x, obstacle.y, msg.header.frame_id.c_str());
  }

  void onGrid(const nav_msgs::msg::OccupancyGrid & msg)
  {
    nav_msgs::msg::OccupancyGrid grid = msg;

    const rclcpp::Time stamp(grid.header.stamp);
    // Clamped so a bag loop or a stalled sim cannot teleport a mover.
    const double dt = last_stamp_ ?
      std::clamp((stamp - *last_stamp_).seconds(), 0.0, 0.5) : 0.0;
    last_stamp_ = stamp;

    if (enabled_ && grid.info.resolution > 0.0 &&
      grid.data.size() == static_cast<size_t>(grid.info.width) * grid.info.height)
    {
      advanceMovers(dt);

      tf2::Transform frame_from_map;
      if (lookupFrameFromMap(grid, frame_from_map)) {
        tf2::Transform frame_from_origin;
        tf2::fromMsg(grid.info.origin, frame_from_origin);
        const tf2::Transform origin_from_map = frame_from_origin.inverse() * frame_from_map;

        for (const auto & obstacle : obstacles_) {
          if (obstacle.anchored() && !reference_.valid()) {
            continue;
          }
          if (overlapsEgo(obstacle, grid.header.frame_id, frame_from_map)) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Skipping '%s': within %.2f m of ego", obstacle.name.c_str(), min_ego_distance_m_);
            continue;
          }
          stampObstacle(grid, obstacle, origin_from_map);
        }
      }
    }

    grid_pub_->publish(grid);
    publishMarkers(grid.header.stamp);
  }

  void advanceMovers(double dt)
  {
    if (!reference_.valid()) {
      return;
    }

    for (auto & obstacle : obstacles_) {
      if (obstacle.kind != ObstacleKind::RacelineMover) {
        continue;
      }
      const double speed =
        obstacle.speed_scale * std::max(0.0, reference_.sampleAtS(obstacle.s).velocity);
      obstacle.s = reference_.wrapS(obstacle.s + speed * dt);
      placeOnRaceline(obstacle);
    }
  }

  bool lookupFrameFromMap(
    const nav_msgs::msg::OccupancyGrid & grid,
    tf2::Transform & frame_from_map)
  {
    frame_from_map.setIdentity();
    if (grid.header.frame_id == map_frame_) {
      return true;
    }

    // Always take the latest available transform.  Matching the grid stamp
    // exactly makes the lookup fail whenever TF and the costmap are even
    // slightly out of sync, and a dropped frame means the obstacle blinks out
    // of the grid entirely.  A few milliseconds of TF staleness is invisible
    // next to that.
    try {
      const auto transform = tf_buffer_->lookupTransform(
        grid.header.frame_id, map_frame_, tf2::TimePointZero);
      tf2::fromMsg(transform.transform, frame_from_map);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform '%s' -> '%s': %s",
        map_frame_.c_str(), grid.header.frame_id.c_str(), ex.what());
      return false;
    }
  }

  // The costmap contract is that ego never appears in its own grid; an obstacle
  // dropped on top of ego breaks every downstream assumption at once.  Only
  // meaningful while the grid is still anchored to base_link.
  bool overlapsEgo(
    const Obstacle & obstacle,
    const std::string & grid_frame,
    const tf2::Transform & frame_from_map) const
  {
    if (grid_frame == map_frame_) {
      return false;
    }
    const tf2::Vector3 center = frame_from_map * tf2::Vector3(obstacle.x, obstacle.y, 0.0);
    return std::hypot(center.x(), center.y()) < min_ego_distance_m_;
  }

  void stampObstacle(
    nav_msgs::msg::OccupancyGrid & grid,
    const Obstacle & obstacle,
    const tf2::Transform & origin_from_map) const
  {
    // Grid-origin-local coordinates, where cell (row, col) spans
    // [col * res, (col + 1) * res) x [row * res, (row + 1) * res).
    const tf2::Vector3 center = origin_from_map * tf2::Vector3(obstacle.x, obstacle.y, 0.0);
    const double yaw = obstacle.yaw + tf2::getYaw(origin_from_map.getRotation());
    const double resolution = grid.info.resolution;
    const double half_length = 0.5 * obstacle.length_m;
    const double half_width = 0.5 * obstacle.width_m;
    const double reach = std::hypot(half_length, half_width);

    const int col_min = std::max(
      0, static_cast<int>(std::floor((center.x() - reach) / resolution)));
    const int col_max = std::min(
      static_cast<int>(grid.info.width) - 1,
      static_cast<int>(std::floor((center.x() + reach) / resolution)));
    const int row_min = std::max(
      0, static_cast<int>(std::floor((center.y() - reach) / resolution)));
    const int row_max = std::min(
      static_cast<int>(grid.info.height) - 1,
      static_cast<int>(std::floor((center.y() + reach) / resolution)));

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    for (int row = row_min; row <= row_max; ++row) {
      for (int col = col_min; col <= col_max; ++col) {
        const double dx = (static_cast<double>(col) + 0.5) * resolution - center.x();
        const double dy = (static_cast<double>(row) + 0.5) * resolution - center.y();
        const double along = dx * cos_yaw + dy * sin_yaw;
        const double across = -dx * sin_yaw + dy * cos_yaw;
        if (std::abs(along) <= half_length && std::abs(across) <= half_width) {
          grid.data[static_cast<size_t>(row) * grid.info.width + col] = obstacle_value_;
        }
      }
    }
  }

  void publishMarkers(const builtin_interfaces::msg::Time & stamp) const
  {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int id = 0;
    for (const auto & obstacle : obstacles_) {
      if (obstacle.anchored() && !reference_.valid()) {
        continue;
      }

      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = map_frame_;
      marker.ns = "injected_obstacles";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = obstacle.x;
      marker.pose.position.y = obstacle.y;
      marker.pose.position.z = 0.15;
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, obstacle.yaw);
      marker.pose.orientation = tf2::toMsg(orientation);
      marker.scale.x = obstacle.length_m;
      marker.scale.y = obstacle.width_m;
      marker.scale.z = 0.30;
      const bool mover = obstacle.kind == ObstacleKind::RacelineMover;
      marker.color.a = 0.8f;
      marker.color.r = mover ? 0.9f : 0.4f;
      marker.color.g = 0.2f;
      marker.color.b = mover ? 0.2f : 0.9f;
      markers.markers.push_back(marker);
    }

    marker_pub_->publish(markers);
  }

  bool enabled_ = true;
  std::string map_frame_;
  std::string ego_frame_;
  int8_t obstacle_value_ = 100;
  double min_ego_distance_m_ = 0.5;
  double clicked_obstacle_size_m_ = 0.30;
  double respawn_min_ego_distance_m_ = 4.0;
  double respawn_lateral_range_m_ = 0.0;

  std::vector<Obstacle> obstacles_;
  local_planning::RacelineReference reference_;
  std::optional<rclcpp::Time> last_stamp_;
  std::mt19937 rng_{std::random_device{}()};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raceline_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr respawn_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapObstacleInjector>());
  rclcpp::shutdown();
  return 0;
}
