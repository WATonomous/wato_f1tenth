#pragma once  // Include guard to prevent multiple inclusions of this header file

#include <rclcpp/rclcpp.hpp>                                 // Core ROS2 C++ API header
#include <sensor_msgs/msg/laser_scan.hpp>                    // Definition for LaserScan messages
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>    // Definition for Ackermann drive command messages
#include <sensor_msgs/msg/trajectories.hpp>                 // Definition for custom trajectory messages
#include <vector>                                            // STL vector container
#include <utility>                                           // STL pair utility
#include <cmath>                                            // Math functions

class reactiveColAvoid : public rclcpp::Node           // Define class reactiveColAvoid inheriting from ROS2 Node
{
public:
  reactiveColAvoid() : Node("gap_publisher") {}  // Constructor declaration

private:
  // LiDAR processing: fills 'gaps' and 'gapsMid'
  void process_LiDAR(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg,  // Incoming LaserScan message pointer
    std::vector<std::pair<double, double>>& gaps,            // Output vector for raw gap data
    std::vector<std::pair<double, double>>& gapsMid);        // Output vector for midpoint gap data

  // Chooses best gap based on subscribed trajectory and publishes drive command
  void chooseBestGap();                                     // Function to select and publish best gap
  double velocity();                                         // Function to handle velocity commands

  // Callbacks
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);  // Callback for LiDAR scans
  void traj_callback(const sensor_msgs::msg::trajectories::SharedPtr traj_msg); // Callback for trajectory updates

  // Subscribers & Publisher handles
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr              scan_sub;       // Subscriber to LiDAR
  rclcpp::Subscription<sensor_msgs::msg::trajectories>::SharedPtr         trajectories;   // Subscriber to trajectory
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;     // Publisher for drive commands

  // State
  sensor_msgs::msg::trajectories                    latest_traj;     // Stores latest trajectory message
  bool                                              traj_received_{false};  // Flag to check if trajectory received
  std::vector<std::pair<double, double>>            gapsMid;         // Vector of midpoints of valid gaps
};
