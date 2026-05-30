#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

class SimObstacleOverlay : public rclcpp::Node
{
public:
  SimObstacleOverlay()
  : Node("sim_obstacle_overlay_node")
  {
    this->declare_parameter<std::string>("raw_grid_topic", "/costmap");
    this->declare_parameter<std::string>("grid_topic", "/occupancy_grid");
    this->declare_parameter<std::string>("clicked_point_topic", "/clicked_point");
    this->declare_parameter<std::string>("marker_topic", "/sim_obstacles/markers");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("obstacle_length_m", 0.4);
    this->declare_parameter<double>("obstacle_width_m", 0.4);
    this->declare_parameter<int>("occupied_value", 100);

    raw_grid_topic_ = this->get_parameter("raw_grid_topic").as_string();
    grid_topic_ = this->get_parameter("grid_topic").as_string();
    clicked_point_topic_ = this->get_parameter("clicked_point_topic").as_string();
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    obstacle_length_m_ = this->get_parameter("obstacle_length_m").as_double();
    obstacle_width_m_ = this->get_parameter("obstacle_width_m").as_double();
    const int occupied_value = static_cast<int>(this->get_parameter("occupied_value").as_int());
    occupied_value_ = static_cast<int8_t>(std::clamp(occupied_value, 0, 100));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(1).transient_local());

    raw_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      raw_grid_topic_, 10,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        last_raw_grid_ = msg;
        publishMergedGrid(*msg);
      });

    const auto clicked_point_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_point_topic_, clicked_point_qos,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        addObstacle(*msg);
      });

    clear_srv_ = this->create_service<std_srvs::srv::Empty>(
      "/sim_obstacles/clear",
      [this](
        const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/) {
        obstacles_.clear();
        publishMarkers();
        if (last_raw_grid_) {
          publishMergedGrid(*last_raw_grid_);
        }
        RCLCPP_INFO(this->get_logger(), "Cleared simulated obstacles");
      });

    publishMarkers();

    RCLCPP_INFO(
      this->get_logger(),
      "Overlaying simulated %.2fm x %.2fm obstacles from %s onto %s",
      obstacle_length_m_, obstacle_width_m_, clicked_point_topic_.c_str(), grid_topic_.c_str());
  }

private:
  struct Obstacle
  {
    double x;
    double y;
  };

  void addObstacle(const geometry_msgs::msg::PointStamped & point)
  {
    geometry_msgs::msg::PointStamped point_in_map;
    if (!point.header.frame_id.empty() && point.header.frame_id != frame_id_) {
      try {
        point_in_map = tf_buffer_->transform(point, frame_id_, tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException & stamped_ex) {
        geometry_msgs::msg::PointStamped latest_point = point;
        latest_point.header.stamp.sec = 0;
        latest_point.header.stamp.nanosec = 0;
        try {
          point_in_map = tf_buffer_->transform(
            latest_point, frame_id_, tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException & latest_ex) {
          RCLCPP_WARN(
            this->get_logger(),
            "Ignoring clicked point in frame '%s'; cannot transform to '%s': %s; latest TF fallback also failed: %s",
            point.header.frame_id.c_str(), frame_id_.c_str(), stamped_ex.what(),
            latest_ex.what());
          return;
        }
      }
    } else {
      point_in_map = point;
      point_in_map.header.frame_id = frame_id_;
    }

    obstacles_.push_back({point_in_map.point.x, point_in_map.point.y});
    publishMarkers();
    if (last_raw_grid_) {
      publishMergedGrid(*last_raw_grid_);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Added simulated obstacle %zu at (%.2f, %.2f) in %s",
      obstacles_.size(), point_in_map.point.x, point_in_map.point.y, frame_id_.c_str());
  }

  void publishMergedGrid(const nav_msgs::msg::OccupancyGrid & raw_grid)
  {
    nav_msgs::msg::OccupancyGrid merged_grid;
    if (!convertToMapGrid(raw_grid, merged_grid)) {
      return;
    }
    stampObstacles(merged_grid);
    grid_pub_->publish(merged_grid);
  }

  bool convertToMapGrid(
    const nav_msgs::msg::OccupancyGrid & raw_grid,
    nav_msgs::msg::OccupancyGrid & map_grid)
  {
    const size_t cell_count =
      static_cast<size_t>(raw_grid.info.width) * static_cast<size_t>(raw_grid.info.height);
    if (raw_grid.header.frame_id.empty() || raw_grid.info.resolution <= 0.0 ||
      raw_grid.info.width == 0 || raw_grid.info.height == 0 ||
      raw_grid.data.size() != cell_count)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring malformed occupancy grid from %s", raw_grid_topic_.c_str());
      return false;
    }

    tf2::Transform raw_frame_to_map;
    raw_frame_to_map.setIdentity();
    if (raw_grid.header.frame_id != frame_id_) {
      geometry_msgs::msg::TransformStamped transform;
      try {
        transform = tf_buffer_->lookupTransform(
          frame_id_, raw_grid.header.frame_id, rclcpp::Time(raw_grid.header.stamp),
          tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException & stamped_ex) {
        try {
          transform = tf_buffer_->lookupTransform(
            frame_id_, raw_grid.header.frame_id, tf2::TimePointZero,
            tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException & latest_ex) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Cannot transform occupancy grid from '%s' to '%s': %s; latest TF fallback also failed: %s",
            raw_grid.header.frame_id.c_str(), frame_id_.c_str(), stamped_ex.what(),
            latest_ex.what());
          return false;
        }
      }
      tf2::fromMsg(transform.transform, raw_frame_to_map);
    }

    tf2::Transform grid_to_raw_frame;
    tf2::fromMsg(raw_grid.info.origin, grid_to_raw_frame);
    const tf2::Transform grid_to_map = raw_frame_to_map * grid_to_raw_frame;
    const tf2::Transform map_to_grid = grid_to_map.inverse();
    const double resolution = raw_grid.info.resolution;
    const double width_m = static_cast<double>(raw_grid.info.width) * resolution;
    const double height_m = static_cast<double>(raw_grid.info.height) * resolution;
    const tf2::Vector3 center_in_map =
      grid_to_map * tf2::Vector3(width_m * 0.5, height_m * 0.5, 0.0);

    map_grid = raw_grid;
    map_grid.header.frame_id = frame_id_;
    map_grid.info.origin.position.x = center_in_map.x() - width_m * 0.5;
    map_grid.info.origin.position.y = center_in_map.y() - height_m * 0.5;
    map_grid.info.origin.position.z = 0.0;
    map_grid.info.origin.orientation.x = 0.0;
    map_grid.info.origin.orientation.y = 0.0;
    map_grid.info.origin.orientation.z = 0.0;
    map_grid.info.origin.orientation.w = 1.0;
    map_grid.data.assign(cell_count, -1);

    for (uint32_t row = 0; row < map_grid.info.height; ++row) {
      for (uint32_t col = 0; col < map_grid.info.width; ++col) {
        const tf2::Vector3 point_in_map(
          map_grid.info.origin.position.x + (static_cast<double>(col) + 0.5) * resolution,
          map_grid.info.origin.position.y + (static_cast<double>(row) + 0.5) * resolution,
          0.0);
        const tf2::Vector3 point_in_grid = map_to_grid * point_in_map;
        const int source_col = static_cast<int>(std::floor(point_in_grid.x() / resolution));
        const int source_row = static_cast<int>(std::floor(point_in_grid.y() / resolution));

        if (source_col >= 0 && source_col < static_cast<int>(raw_grid.info.width) &&
          source_row >= 0 && source_row < static_cast<int>(raw_grid.info.height))
        {
          map_grid.data[static_cast<size_t>(row) * map_grid.info.width + col] =
            raw_grid.data[static_cast<size_t>(source_row) * raw_grid.info.width + source_col];
        }
      }
    }

    return true;
  }

  void stampObstacles(nav_msgs::msg::OccupancyGrid & grid) const
  {
    if (grid.info.resolution <= 0.0 || grid.info.width == 0 || grid.info.height == 0) {
      return;
    }

    for (const auto & obstacle : obstacles_) {
      stampObstacle(grid, obstacle);
    }
  }

  void stampObstacle(nav_msgs::msg::OccupancyGrid & grid, const Obstacle & obstacle) const
  {
    const double half_length = obstacle_length_m_ * 0.5;
    const double half_width = obstacle_width_m_ * 0.5;
    const double origin_x = grid.info.origin.position.x;
    const double origin_y = grid.info.origin.position.y;
    const double resolution = grid.info.resolution;
    const int width = static_cast<int>(grid.info.width);
    const int height = static_cast<int>(grid.info.height);

    int min_col = static_cast<int>(std::floor((obstacle.x - half_width - origin_x) / resolution));
    int max_col = static_cast<int>(std::floor((obstacle.x + half_width - origin_x) / resolution));
    int min_row = static_cast<int>(std::floor((obstacle.y - half_length - origin_y) / resolution));
    int max_row = static_cast<int>(std::floor((obstacle.y + half_length - origin_y) / resolution));

    if (max_col < 0 || max_row < 0 || min_col >= width || min_row >= height) {
      return;
    }

    min_col = std::clamp(min_col, 0, width - 1);
    max_col = std::clamp(max_col, 0, width - 1);
    min_row = std::clamp(min_row, 0, height - 1);
    max_row = std::clamp(max_row, 0, height - 1);

    for (int row = min_row; row <= max_row; ++row) {
      for (int col = min_col; col <= max_col; ++col) {
        const double cell_x = origin_x + (static_cast<double>(col) + 0.5) * resolution;
        const double cell_y = origin_y + (static_cast<double>(row) + 0.5) * resolution;
        if (std::abs(cell_x - obstacle.x) <= half_width &&
          std::abs(cell_y - obstacle.y) <= half_length)
        {
          grid.data[static_cast<size_t>(row * width + col)] = occupied_value_;
        }
      }
    }
  }

  void publishMarkers() const
  {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = frame_id_;
    clear.header.stamp = this->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    for (size_t i = 0; i < obstacles_.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = clear.header.stamp;
      marker.ns = "sim_obstacles";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = obstacles_[i].x;
      marker.pose.position.y = obstacles_[i].y;
      marker.pose.position.z = 0.08;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacle_width_m_;
      marker.scale.y = obstacle_length_m_;
      marker.scale.z = 0.16;
      marker.color.r = 1.0;
      marker.color.g = 0.45;
      marker.color.b = 0.05;
      marker.color.a = 0.75;
      markers.markers.push_back(marker);
    }

    marker_pub_->publish(markers);
  }

  std::string raw_grid_topic_;
  std::string grid_topic_;
  std::string clicked_point_topic_;
  std::string marker_topic_;
  std::string frame_id_;
  double obstacle_length_m_;
  double obstacle_width_m_;
  int8_t occupied_value_;

  std::vector<Obstacle> obstacles_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_raw_grid_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimObstacleOverlay>());
  rclcpp::shutdown();
  return 0;
}
