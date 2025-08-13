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
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub;

    //subs
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_mon;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_steering_mon;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    //debug (can't have this active during race or actual testing)
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub;

    //functions
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void laser_callbackv2(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    //data
    double current_speed;
    ackermann_msgs::msg::AckermannDriveStamped current_ackermann_msg, current_steering;
    bool is_breaking = false;
    int prock_counter = 0;
    float desired_speed;

    //parameters
    std::string ackermann_output_topic, ackermann_mon_topic, laser_topic, odom_topic, steering_topic;
    double ttc1, ttc2, ttc3;
    double ttc_throtel_1, ttc_throtel_2, ttc_throtel_3;
    double speed_threshold, look_ofset;
    double MAX_SPEED;
    int alarm_threshold; 

};


#endif