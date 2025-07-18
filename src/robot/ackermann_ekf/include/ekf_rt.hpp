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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/LinearMath/Quaternion.h>

//use of use
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

enum state {
    x = 0, 
    y = 1, 
    theta = 2, 
    v = 3, 
    theta_dot = 4,
    ax = 5,
    ay = 6
};

struct mu_and_sigma {
    vector7d new_state;
    matrix7d new_covariance;
    rclcpp::Time current;
};

class EKF_RT : public rclcpp::Node {
public:

    EKF_RT();

private:          

    //publishers

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub;

    //subscribers

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    //synced messages for throtel and steering input
    
    message_filters::Subscriber<std_msgs::msg::Float32> steering_sub;
    message_filters::Subscriber<std_msgs::msg::Float32> throtel_sub;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
                    std_msgs::msg::Float32,std_msgs::msg::Float32>;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;


    //helper functions
    double calculate_delta_t(rclcpp::Time current, rclcpp::Time prev);
    void initalize_params ();
    void is_time_init ();

    //callback function
    void control_input_callback(std_msgs::msg::Float32::SharedPtr throtel,std_msgs::msg::Float32::SharedPtr steering);
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr odom_msg);

    //publishing
    void publish_state ();

    //prediction step
    vector7d model_update(const vector7d &mu_prev, const double &steering_input, const double &throtel_input, double &dt); 
    matrix7d jacobian_g_update (const vector7d &mu_prev, const double &steering_input, const double &throtel_input, double &dt);
    matrix7d predict_sigma (const matrix7d &sigma_prev, const matrix7d &jacobian_g); 
    mu_and_sigma prediction_step(const vector7d &mu_prev, const matrix7d &sigma_prev, 
        const std_msgs::msg::Float32 &steering_input, const std_msgs::msg::Float32 &throtel_input);

    //correction step
    vector4d imu_state_mapper(const vector7d &mu_predicted);
    vector5d odom_state_mapper(const vector7d &mu_predicted);
    vector4d imu_observation_maker(const sensor_msgs::msg::Imu &observation);
    vector5d odom_observation_maker(const nav_msgs::msg::Odometry &observation);

    //data
    std_msgs::msg::Float32 prev_steering, prev_throtel;
    vector7d mu;
    matrix7d sigma_t;
    rclcpp::Time prev_update_time;

    bool init_time = false;

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
    std::string odom_topic, ekf_topic, imu_topic, throtel_topic, steering_topic, child_frame, header_frame;
    double wheel_base, max_speed;
};



#endif