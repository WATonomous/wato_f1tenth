#include "rclcpp/rclcpp.hpp"
#include "emergency_braking.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <functional>

EmergencyBraking::EmergencyBraking() : Node("emergency_braking_node"), current_speed_(0.0) {
    // subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&EmergencyBraking::scan_callback, this, std::placeholders::_1)
    );
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 10, std::bind(&EmergencyBraking::odom_callback, this, std::placeholders::_1)
    );
    // publishers
    brake_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    RCLCPP_INFO(this->get_logger(), "node started");
}

void EmergencyBraking::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "-------------------------");
    RCLCPP_INFO(this->get_logger(), "odom_callback called");

    // Log the entire twist structure
    // RCLCPP_INFO(this->get_logger(), "  msg->twist: linear=[x: %f, y: %f, z: %f], angular=[x: %f, y: %f, z: %f]",
    //              msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
    //              msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

    double linear_x_velocity = msg->twist.twist.linear.x;
    RCLCPP_INFO(this->get_logger(), "linear x velocity: %f", linear_x_velocity);
    current_speed_ = linear_x_velocity;
    RCLCPP_INFO(this->get_logger(), "current_speed: %f", current_speed_);
    RCLCPP_INFO(this->get_logger(), "-------------------------");
}

void EmergencyBraking::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "-------------------------");
    RCLCPP_INFO(this->get_logger(), "scan callback");
    for(size_t i = 0; i < msg->ranges.size(); i++){
        double obs_dist = msg->ranges[i];
        // ensure distance isn't infinite and not a number
        if(!std::isinf(obs_dist) && !std::isnan(obs_dist)){
            RCLCPP_INFO(this->get_logger(), "distance is valid");
            double ttc = obs_dist / std::max(0.0, (current_speed_ * cos(i * msg->angle_increment - PI)));
            // publish brake msg if ttc is less than safe threshold
            if(current_speed_ < 0 && (ttc < (abs(current_speed_) / 8.26 + 0.12))){
                RCLCPP_INFO(this->get_logger(), "current_speed_ < 0");
                apply_brake();
            }else if(current_speed_ > 0 && (ttc < (current_speed_ / 8.26 + 0.02))){
                RCLCPP_INFO(this->get_logger(), "current_speed_ > 0");
                apply_brake();
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "range 0: %f", msg->ranges[0]);
    RCLCPP_INFO(this->get_logger(), "-------------------------");
}

void EmergencyBraking::apply_brake(){
    RCLCPP_INFO(this->get_logger(), "-------------------------");
    RCLCPP_INFO(this->get_logger(), "-------------------------");
    RCLCPP_INFO(this->get_logger(), "apply brake. current speed: %f", current_speed_);
    RCLCPP_INFO(this->get_logger(), "-------------------------");
    RCLCPP_INFO(this->get_logger(), "-------------------------");
    auto brake_msg = ackermann_msgs::msg::AckermannDriveStamped();
    brake_msg.drive.speed = 0.0;
    brake_pub_->publish(brake_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmergencyBraking>());
    rclcpp::shutdown();
    return 0;
}