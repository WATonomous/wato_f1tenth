#ifndef EKF_NODE_
#define EKF_NODE_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using matrix7d = Eigen::Matrix<double,7,7>;
using vec7d = Eigen::Matrix<double,7,1>;

enum State_space {
    X,Y,THETA,V,THETA_DOT,AX,AY
};


class EKF_NODE : public rclcpp::Node {
public: 

    EKF_NODE();

private:

    //subscribers

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr Throtel_sub;

    //publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub;

    //functions

    void odomCallBack(nav_msgs::msg::Odometry::SharedPtr msg);
    void imuCallBack (sensor_msgs::msg::Imu::SharedPtr msg);
    void steeringCallBack(std_msgs::msg::Float32::SharedPtr msg);
    void publishOdom(const nav_msgs::msg::Odometry &odom);
    void throtelCallBack(std_msgs::msg::Float32::SharedPtr msg);

    void ekf_pass(const nav_msgs::msg::Odometry &odom, const vec7d &mu_p,const matrix7d &covariance_p,vec7d &new_mu, matrix7d &new_sigma_t);
    vec7d observationCreator(nav_msgs::msg::Odometry wheel_odom, sensor_msgs::msg::Imu imu);
    vec7d observationMapper(const vec7d &predicted_state);
    matrix7d calculateJacobianG(const vec7d &current_state,const std_msgs::msg::Float32 &current_steering);
    vec7d modelUpdate (const vec7d &current_state,const std_msgs::msg::Float32 &current_steering);
    vec7d modelUpdate2 (const vec7d &current_state,const std_msgs::msg::Float32 &steering_angle);
    matrix7d calculateJacobianG2(const vec7d &previous_state,const std_msgs::msg::Float32 &steering_angle);
    vec7d observationCreator2(const nav_msgs::msg::Odometry wheel_odom,const sensor_msgs::msg::Imu imu);

    void initalize();

    //data

    vec7d *mu; 
    matrix7d *sigma_t;
    matrix7d R; // process noise
    matrix7d Q; // sensor noise
    matrix7d H; // observation matrix jacobian
    matrix7d I7; //7x7 identidy atrix 

    sensor_msgs::msg::Imu current_imu;
    std_msgs::msg::Float32 current_steering;
    std_msgs::msg::Float32 current_throtel;

    bool intalized_time = false;
    bool check_one = false;

    int count;

    double DT;
    rclcpp::Time t_previous;

    //constants
    const double L_WB = 0.3240; // the wheel base lenight
    const double MAX_VELOCITY = 22.88; // the max speed of the car

    //filter vars
    double filter_ax = 0, filter_ay = 0;
    double filter_x_gain = 0.98, filter_y_gain = 0.98;

};

#endif