#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class OccupancyGridFrameAdapter : public rclcpp::Node
{
public:
  OccupancyGridFrameAdapter()
  : Node("occupancy_grid_frame_adapter_node")
  {
    this->declare_parameter<std::string>("raw_grid_topic", "/costmap");
    this->declare_parameter<std::string>("grid_topic", "/occupancy_grid");
    this->declare_parameter<std::string>("frame_id", "map");

    raw_grid_topic_ = this->get_parameter("raw_grid_topic").as_string();
    grid_topic_ = this->get_parameter("grid_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, 10);
    raw_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      raw_grid_topic_, 10,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        publishGrid(*msg);
      });

  }

private:
  static bool isValidOccupancyGrid(const nav_msgs::msg::OccupancyGrid & grid)
  {
    if (grid.header.frame_id.empty() || grid.info.resolution <= 0.0 ||
      grid.info.width == 0 || grid.info.height == 0)
    {
      return false;
    }

    const size_t width = grid.info.width;
    const size_t height = grid.info.height;
    if (width > std::numeric_limits<size_t>::max() / height) {
      return false;
    }

    return grid.data.size() == width * height;
  }

  static bool projectOccupancyGrid(
    const nav_msgs::msg::OccupancyGrid & source_grid,
    const tf2::Transform & source_frame_to_target,
    const std::string & target_frame_id,
    nav_msgs::msg::OccupancyGrid & target_grid)
  {
    if (!isValidOccupancyGrid(source_grid) || target_frame_id.empty()) {
      return false;
    }

    tf2::Transform grid_to_source_frame;
    tf2::fromMsg(source_grid.info.origin, grid_to_source_frame);
    const tf2::Transform grid_to_target = source_frame_to_target * grid_to_source_frame;
    const tf2::Transform target_to_grid = grid_to_target.inverse();
    const double resolution = source_grid.info.resolution;
    const double width_m = static_cast<double>(source_grid.info.width) * resolution;
    const double height_m = static_cast<double>(source_grid.info.height) * resolution;
    const tf2::Vector3 center_in_target =
      grid_to_target * tf2::Vector3(width_m * 0.5, height_m * 0.5, 0.0);

    target_grid = source_grid;
    target_grid.header.frame_id = target_frame_id;
    target_grid.info.origin.position.x = center_in_target.x() - width_m * 0.5;
    target_grid.info.origin.position.y = center_in_target.y() - height_m * 0.5;
    target_grid.info.origin.position.z = 0.0;
    target_grid.info.origin.orientation.x = 0.0;
    target_grid.info.origin.orientation.y = 0.0;
    target_grid.info.origin.orientation.z = 0.0;
    target_grid.info.origin.orientation.w = 1.0;
    target_grid.data.assign(source_grid.data.size(), -1);

    for (uint32_t row = 0; row < target_grid.info.height; ++row) {
      for (uint32_t col = 0; col < target_grid.info.width; ++col) {
        const tf2::Vector3 point_in_target(
          target_grid.info.origin.position.x + (static_cast<double>(col) + 0.5) * resolution,
          target_grid.info.origin.position.y + (static_cast<double>(row) + 0.5) * resolution,
          0.0);
        const tf2::Vector3 point_in_grid = target_to_grid * point_in_target;
        const int source_col = static_cast<int>(std::floor(point_in_grid.x() / resolution));
        const int source_row = static_cast<int>(std::floor(point_in_grid.y() / resolution));

        if (source_col >= 0 && source_col < static_cast<int>(source_grid.info.width) &&
          source_row >= 0 && source_row < static_cast<int>(source_grid.info.height))
        {
          target_grid.data[static_cast<size_t>(row) * target_grid.info.width + col] =
            source_grid.data[static_cast<size_t>(source_row) * source_grid.info.width + source_col];
        }
      }
    }

    return true;
  }

  void publishGrid(const nav_msgs::msg::OccupancyGrid & raw_grid)
  {
    if (!isValidOccupancyGrid(raw_grid)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring malformed occupancy grid from %s", raw_grid_topic_.c_str());
      return;
    }

    tf2::Transform raw_frame_to_target;
    raw_frame_to_target.setIdentity();
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
            this->get_logger(),
            *this->get_clock(), 2000,
            "Cannot transform occupancy grid from '%s' to '%s': %s; latest TF fallback also failed: %s",
            raw_grid.header.frame_id.c_str(), frame_id_.c_str(), stamped_ex.what(),
            latest_ex.what());
          return;
        }
      }
      tf2::fromMsg(transform.transform, raw_frame_to_target);
    }

    nav_msgs::msg::OccupancyGrid target_grid;
    if (!projectOccupancyGrid(
        raw_grid, raw_frame_to_target, frame_id_, target_grid))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring occupancy grid from %s because it cannot be projected to frame '%s'",
        raw_grid_topic_.c_str(), frame_id_.c_str());
      return;
    }

    grid_pub_->publish(target_grid);
  }

  std::string raw_grid_topic_;
  std::string grid_topic_;
  std::string frame_id_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_grid_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridFrameAdapter>());
  rclcpp::shutdown();
  return 0;
}
