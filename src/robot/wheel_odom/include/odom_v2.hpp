#ifndef ODOM_HPP_
#define ODOM_HPP_


#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/LinearMath/Quaternion.h>

class ODOM : public rclcpp::Node {
public:

    ODOM();

private:

    //subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub;
    message_filters::Subscriber<sensor_msgs::msg::JointState> left_sub;
    message_filters::Subscriber<sensor_msgs::msg::JointState> right_sub;

    //message syncing
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::JointState,sensor_msgs::msg::JointState>;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    //publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    //data
    std_msgs::msg::Float32 steering_data;
    double x,y,theta,velocity,angular_velocity;
    sensor_msgs::msg::JointState left_prev,right_prev;
    bool intalize_prev = false;

    //functions
    void steering_callback (std_msgs::msg::Float32::SharedPtr msg);
    void odom_callback (const sensor_msgs::msg::JointState::ConstSharedPtr &left,
        const sensor_msgs::msg::JointState::ConstSharedPtr &right);
    void publish_odom ();

    //parameters
    std::string right_topic,left_topic,odom_topic,steering_topic,header_frame,child_frame;
    double wheel_radius,wheel_base;

};

#endif  