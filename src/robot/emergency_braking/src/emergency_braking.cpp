#include <cmath>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class EmergencyBraking : public rclcpp::Node {

   public:
        EmergencyBraking() : Node("emergency_braking_node") {
            brake_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&EmergencyBraking::scan_callback, this, _1));
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom", 10, std::bind(&EmergencyBraking::odom_callback, this, _1));
        }

   private:
        double speed = 0.0;

        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr brake_pub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            this->speed = msg->twist.twist.linear.x;
        }

        void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
            // calculate TTC
            bool brake = false;
            double threshold = 1;
            for (std::size_t i = 0; i < scan_msg->ranges.size(); i++) {
                double range = scan_msg->ranges[i];
                bool not_valid = std::isnan(range) || range > scan_msg->range_max || range < scan_msg->range_min;
                if (not_valid) {
                    continue;
                }   
                double component = this->speed * std::cos(scan_msg->angle_min + (double)i * scan_msg->angle_increment);
                // ensure speed is not 0 (don't divide by 0)
                double v = std::max(component, 0.001);
                double ttc = range / v;
                RCLCPP_INFO(this->get_logger(), "ttc: %f", ttc);
                if (ttc < threshold) {
                    brake = true;
                    break;
                }
            }
            // publish braking message by setting speed to 0
            if (brake) {
                auto brake_msg = ackermann_msgs::msg::AckermannDriveStamped();
                brake_msg.drive.speed = 0.0;
                RCLCPP_INFO(this->get_logger(), "stop car");  
                this->brake_pub_->publish(brake_msg);
            }
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmergencyBraking>());
    rclcpp::shutdown();
    return 0;
}