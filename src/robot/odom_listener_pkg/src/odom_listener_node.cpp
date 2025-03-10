#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomListenerNode : public rclcpp::Node {
public:
    OdomListenerNode() : Node("odom_listener_node") {
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&OdomListenerNode::odom_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Odom listener node started, subscribing to /ego_racecar/odom");

    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double linear_x_velocity = msg->twist.twist.linear.x;
        RCLCPP_INFO(this->get_logger(), "Received Odometry: Linear Velocity X = %f", linear_x_velocity);
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomListenerNode>());
    rclcpp::shutdown();
    return 0;
}