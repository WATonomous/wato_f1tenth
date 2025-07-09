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
using vector7d = Eigen::Matrix<double,7,1>;
using matrix7d = Eigen::Matrix<double,7,7>;

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
    double calculate_delta_t(rclcpp::Time t_prev, rclcpp::Time t_now);
    void initalize_params ();

    //callback function
    void control_input_callback(std_msgs::msg::Float32::SharedPtr throtel,std_msgs::msg::Float32::SharedPtr steering);
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr odom_msg);

    //publishing
    void publish_state ();

    //prediction step
    vector7d model_update(const vector7d &mu_prev, const double &steering_input, const double &throtel_input, double &dt); 
    matrix7d jacobian_g_update (const vector7d &mu_prev, const double &steering_input, const double &throtel_input, double &dt);
    void prediction_step(const vector7d &mu_prev, const matrix7d &sigma_t_prev, const double steering_input, 
                    const double throtel_input, vector7d &predicted_mu, matrix7d &predicted_sigma_t);

    //correction step
    

    //data
    std_msgs::msg::Float32 prev_steering, prev_throtel;
    vector7d mu;
    matrix7d sigma_t;
    rclcpp::Time prev_update_time;

    bool init_time = false;

    //parameters
    std::string odom_topic, ekf_topic, imu_topic, throtel_topic, steering_topic, child_frame, header_frame;
    double wheel_base;
};



#endif