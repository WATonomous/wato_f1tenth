#ifndef EBREAK_NODE_HPP_
#define EBREAK_NODE_HPP_

#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class EBREAK_NODE : public rclcpp::Node {
public:

    EBREAK_NODE();

private: 

    //pubs 
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throtel_pub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub;

    //subs
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throtel_mom_sub;

    //functions
    void laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);

    //data
    double current_speed,current_throtel;

    //parameters
    std::string throtel_topic,throtel_command_topic, laser_topic, odom_topic, ackerman_topic;
    double ttc1, ttc2, ttc3;
    double ttc_throtel_1, ttc_throtel_2, ttc_throtel_3;
    double speed_threshold;
    double MAX_SPEED;
    int alarm_threshold; 

};


#endif