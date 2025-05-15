#ifndef SAFETY_SYSTEM_NODE_HPP_
#define SAFETY_SYSTEM_NODE_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

struct Parameters {
    int max_angle,min_angle;
    float TTC_stage1,TTC_stage2,TTC_stage3;
};

class SafetyNode : public rclcpp::Node {
public:
    //funtions
    SafetyNode();

    //call back
    void laiderCallBack(const sensor_msgs::msg::LaserScan::SharedPtr laider_msg);
private:
    //publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laider_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vel_pub;

    //data
    sensor_msgs::msg::Imu::SharedPtr imu_data;
    rclcpp::Time last_time;

    //parameters
    Parameters pram;

};
#endif  
