#ifndef SAFETY_SYSTEM_NODE_HPP_
#define SAFETY_SYSTEM_NODE_HPP_

#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

struct Parameters {
    int alarm_threshold,debounce_threshold;
    double TTC_stage1,TTC_stage2,TTC_stage3,WHEEL_RADIUS,TTC1_Throtel,TTC2_Throtel,TTC3_Throtel;
    double alpha,encoder_bias,lp_factor;//must be between 0 and 1
    double velocity_threshold, acell_threshold;
};

class SafetyNode : public rclcpp::Node {
public:
    //funtions
    SafetyNode();

    //call back
    void laiderCallBack(const sensor_msgs::msg::LaserScan::SharedPtr laider_msg);
    void imuCallBack(const sensor_msgs::msg::Imu::SharedPtr data);
    void findEncoderVelocity();
private:
    //publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laider_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_encoder_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_encoder_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throtel_monitor_sub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throtel_input_pub;

    //data
    sensor_msgs::msg::JointState::SharedPtr left_encoder_data;
    sensor_msgs::msg::JointState::SharedPtr right_encoder_data;

    rclcpp::Time last_imu_time,last_encoder_time;
    double imu_velocity,encoder_velocity;
    double encoder_last_rad,last_accl;
    std_msgs::msg::Float32 curret_throtel;
    int debounce_counter = 0;

    //parameters
    Parameters pram;

};
#endif  
