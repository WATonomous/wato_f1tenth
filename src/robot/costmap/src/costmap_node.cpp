#include "costmap_node.hpp"

using namespace std::chrono_literals;

CostmapNode::CostmapNode() : Node("occupancy_grid_generator")
{
  // Declare parameters
  this->declare_parameter<double>("grid_width", 20.0);
  this->declare_parameter<double>("grid_height", 20.0);
  this->declare_parameter<double>("resolution", 0.1);
  this->declare_parameter<std::string>("robot_frame", "base_link");
  this->declare_parameter<std::string>("scan_topic", "/scan");
  this->declare_parameter<std::string>("output_topic", "/costmap");
  this->declare_parameter<int>("obstacle_value", 100);
  this->declare_parameter<int>("free_value", 0);
  this->declare_parameter<int>("unknown_value", -1);

  // Read parameters
  grid_width_ = this->get_parameter("grid_width").as_double();
  grid_height_ = this->get_parameter("grid_height").as_double();
  resolution_ = this->get_parameter("resolution").as_double();
  robot_frame_ = this->get_parameter("robot_frame").as_string();
  std::string scan_topic = this->get_parameter("scan_topic").as_string();
  std::string output_topic = this->get_parameter("output_topic").as_string();
  obstacle_value_ = static_cast<int8_t>(this->get_parameter("obstacle_value").as_int());
  free_value_ = static_cast<int8_t>(this->get_parameter("free_value").as_int());
  unknown_value_ = static_cast<int8_t>(this->get_parameter("unknown_value").as_int());

  // Compute grid dimensions in cells
  grid_cols_ = static_cast<uint32_t>(grid_width_ / resolution_);
  grid_rows_ = static_cast<uint32_t>(grid_height_ / resolution_);

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic, 10);

  // Subscribers — publish a new grid on every incoming scan
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 5,
      std::bind(&CostmapNode::scan_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
              "Costmap node initialized: %.1fm x %.1fm grid, %.2fm resolution (%u x %u cells)",
              grid_width_, grid_height_, resolution_, grid_cols_, grid_rows_);
}

void CostmapNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  publish_costmap(msg);
}

void CostmapNode::publish_costmap(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
{
  // Grid origin: centered on base_link (0,0), offset by half the grid size
  double origin_x = -grid_width_ / 2.0;
  double origin_y = -grid_height_ / 2.0;

  // Initialize the occupancy grid message
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.stamp = this->now();
  grid_msg.header.frame_id = robot_frame_;

  grid_msg.info.resolution = static_cast<float>(resolution_);
  grid_msg.info.width = grid_cols_;
  grid_msg.info.height = grid_rows_;
  grid_msg.info.origin.position.x = origin_x;
  grid_msg.info.origin.position.y = origin_y;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  // Fill grid with unknown
  grid_msg.data.assign(grid_cols_ * grid_rows_, unknown_value_);

  // Look up transform from laser frame to base_link frame
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
        robot_frame_,
        scan->header.frame_id,
        tf2::TimePointZero,
        100ms);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Transform lookup failed: %s", ex.what());
    return;
  }

  // Get laser origin in base_link frame
  geometry_msgs::msg::PointStamped laser_origin_in_laser;
  laser_origin_in_laser.header = scan->header;
  laser_origin_in_laser.point.x = 0.0;
  laser_origin_in_laser.point.y = 0.0;
  laser_origin_in_laser.point.z = 0.0;

  geometry_msgs::msg::PointStamped laser_origin_in_base;
  tf2::doTransform(laser_origin_in_laser, laser_origin_in_base, transform_stamped);

  double laser_x = laser_origin_in_base.point.x;
  double laser_y = laser_origin_in_base.point.y;

  // Process each laser ray
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    float range = scan->ranges[i];

    if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
      continue;
    }

    float angle = scan->angle_min + i * scan->angle_increment;

    // Compute the hit point in the laser frame, then transform to base_link
    geometry_msgs::msg::PointStamped hit_in_laser, hit_in_base;
    hit_in_laser.header = scan->header;
    hit_in_laser.point.x = range * std::cos(angle);
    hit_in_laser.point.y = range * std::sin(angle);
    hit_in_laser.point.z = 0.0;

    try {
      tf2::doTransform(hit_in_laser, hit_in_base, transform_stamped);
    } catch (const tf2::TransformException &ex) {
      continue;
    }

    double hit_x = hit_in_base.point.x;
    double hit_y = hit_in_base.point.y;

    // Ray trace from laser origin to hit point using Bresenham's algorithm
    int x0 = static_cast<int>((laser_x - origin_x) / resolution_);
    int y0 = static_cast<int>((laser_y - origin_y) / resolution_);
    int x1 = static_cast<int>((hit_x - origin_x) / resolution_);
    int y1 = static_cast<int>((hit_y - origin_y) / resolution_);

    // Mark free cells along the ray
    bresenham(x0, y0, x1, y1, grid_msg.data);

    // Mark the hit cell as occupied
    if (x1 >= 0 && x1 < static_cast<int>(grid_cols_) &&
        y1 >= 0 && y1 < static_cast<int>(grid_rows_)) {
      grid_msg.data[y1 * grid_cols_ + x1] = obstacle_value_;
    }
  }

  costmap_pub_->publish(grid_msg);
}

void CostmapNode::bresenham(int x0, int y0, int x1, int y1,
                             std::vector<int8_t> &grid_data)
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    // Stop before the endpoint (that cell is the obstacle)
    if (x0 == x1 && y0 == y1) {
      break;
    }

    // Mark cell as free if within bounds
    if (x0 >= 0 && x0 < static_cast<int>(grid_cols_) &&
        y0 >= 0 && y0 < static_cast<int>(grid_rows_)) {
      size_t idx = y0 * grid_cols_ + x0;
      // Only overwrite unknown cells with free (don't clear obstacles)
      if (grid_data[idx] == unknown_value_) {
        grid_data[idx] = free_value_;
      }
    }

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
