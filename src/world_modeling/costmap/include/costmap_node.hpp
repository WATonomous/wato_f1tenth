/*
 * Author: Jordan Khatri
 * Last Modified: 2026-04-02
 *
 * Occupancy Grid Generator Node
 *
 * This node subscribes to LiDAR LaserScan messages and produces a 2D
 * occupancy grid (nav_msgs/OccupancyGrid) centered on the robot's base_link
 * frame. Each incoming scan triggers a fresh grid publication so the costmap
 * stays in sync with sensor data at full LiDAR rate.
 *
 * Implementation overview:
 *   - On each LaserScan callback the node allocates a grid filled with
 *     "unknown" cells, looks up the laser -> base_link transform via TF2,
 *     then iterates over every ray in the scan.
 *   - For each valid ray it uses Bresenham's line algorithm to trace from
 *     the laser origin to the hit point, marking traversed cells as free
 *     and the endpoint cell as occupied.
 *   - Grid dimensions, resolution, frame IDs, and topic names are all
 *     configurable through ROS parameters (see config/params.yaml).
 *
 * Testing / Verification:
 *   - Verified in the AutoDRIVE simulator with Foxglove Studio. The
 *     published /costmap topic was visualized as an occupancy grid overlay
 *     and compared against the raw /scan data to confirm correct obstacle
 *     placement, free-space clearing, and frame alignment.
 */

#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class CostmapNode : public rclcpp::Node
{
public:
  CostmapNode();

private:
  // Parameters
  double grid_width_;
  double grid_height_;
  double resolution_;
  std::string robot_frame_;
  int8_t obstacle_value_;
  int8_t free_value_;
  int8_t unknown_value_;
  uint32_t grid_cols_;
  uint32_t grid_rows_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publish_costmap(const sensor_msgs::msg::LaserScan::SharedPtr &scan);
  void bresenham(int x0, int y0, int x1, int y1,
                 std::vector<int8_t> &grid_data);
};

#endif
