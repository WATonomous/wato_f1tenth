#ifndef EMERGENCY_BRAKING_HPP_
#define EMERGENCY_BRAKING_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


#define PI 3.14159

// class that inherits from rclcpp::Node
class EmergencyBraking : public rclcpp::Node{
    public: 
        // constructor
        EmergencyBraking();
    private:
        double current_speed_;
        // publishers
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr brake_pub_;
        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        // callback functions
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg); 
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        // functions
        void apply_brake();
        std::vector<double> compute_ittc(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif