#ifndef EKF_
#define EKF_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using vector4d = Eigen::Matrix<double,4,1>;
using vector5d = Eigen::Matrix<double,5,1>;
using vector7d = Eigen::Matrix<double,7,1>;
using matrix7d = Eigen::Matrix<double,7,7>;
using matrix5d = Eigen::Matrix<double,5,5>;
using matrix4d = Eigen::Matrix<double,4,4>;
using matrix5x7d = Eigen::Matrix<double,5,7>;
using matrix4x7d = Eigen::Matrix<double,4,7>;
using matrix7x5d = Eigen::Matrix<double,7,5>;
using matrix7x4d = Eigen::Matrix<double,7,4>;

enum state_index {
    x = 0, 
    y = 1, 
    theta = 2, 
    v = 3, 
    theta_dot = 4,
    ax = 5,
    ay = 6
};

struct prediction {
    vector7d mu;
    matrix7d sigma_t;
    rclcpp::Time calc_time;
    bool did_error;
};

/*
the main goal of the ekf if to improve the accuracy of the speed and orientation (yaw) estimate
there is no need to try and correct or impove the model furthure, as making it more complex will 
result in a higer computatinal cost
in the future, it might be possible to levrage something like compute shaders to 
make a more sophictated model work, but that will end up being a seperate project entierly 
*/
class EKF : public rclcpp::Node {
public:

    EKF();
    
private:

    //publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_pub;

    //subscribers
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    //helper functions
    void control_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr input);
    void imu_callback (const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void odom_callback (const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    vector7d model_update (const vector7d &mu_prev,const ackermann_msgs::msg::AckermannDriveStamped &control_input, double dt_);
    matrix7d predict_sigma (const matrix7d &sigma_prev, const matrix7d &jacobian_g); 
    matrix7d calc_jacobian_g (const vector7d &mu_prev, double dt_);
    prediction prediction_step (const vector7d &mu_prev, const matrix7d &sigma_prev, const ackermann_msgs::msg::AckermannDriveStamped &control_input);

    vector4d imu_state_mapper(const vector7d &mu_predicted);
    vector5d odom_state_mapper(const vector7d &mu_predicted);
    vector4d imu_observation_maker(const sensor_msgs::msg::Imu &observation);
    vector5d odom_observation_maker(const nav_msgs::msg::Odometry &observation);

    double velocity_lowPass(double current_speed);

    void init_time ();
    void init_params();
    double calc_dt (rclcpp::Time &current_time,rclcpp::Time &prev_time);
    void publish_state();
    void publish_odom();
    void publish_tf();

    //data
    vector7d mu;
    matrix7d sigma_t;
    rclcpp::Time prev_update_time;
    ackermann_msgs::msg::AckermannDriveStamped prev_input;
    double prev_speed;

    //process noise {x , y , theta, v, theta_dot, ax , ay}
    matrix7d R;

    //sensor noise odom {x, y , theta , v, thata_dot}
    matrix5d Q_odom;

    //sensor noise imu {theta , theta_dot, ax, ay}
    matrix4d Q_imu;

    //H matrix for sensor model 
    matrix4x7d H_imu; // 4 x 7 matrix 
    matrix5x7d H_odom; // 5 / 7 matrix 

    //Identity matrix 
    matrix7d I7; 

    //parameters
    std::string odom_topic, ekf_topic, imu_topic, ackermann_topic, child_frame, header_frame;
    double wheel_base;
    bool should_pub_tf;
    double alpha;

    //tf2 broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    //initalization
    bool time_init = false;
    bool state_init = false;

    //debug
    int error_counter = 0;
    int sucess_counter = 0;
    int sucess_counter_imu = 0;
    int sucess_counter_odom = 0;
    bool debug_mode;
};

#include <tf2/LinearMath/Quaternion.h>
#endif