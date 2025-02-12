#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <vector>
#include <cmath>

class EmergencyBraking : public rclcpp::Node {
    public:
        EmergencyBraking() : Node("emergency_braking"), current_speed_(0.0) {
            // Subscriber for LaserScan
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&EmergencyBraking::scan_callback, this, std::placeholders::_1)
            );

            // Subscriber for Odometry
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/ego_racecar/odom", 10, std::bind(&EmergencyBraking::odom_callback, this, std::placeholders::_1)
            );

            // Publisher for braking (control car speed)
            brake_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                "/drive", 10
            );

            RCLCPP_INFO(this->get_logger(), "node started");
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr brake_pub_;
        
        double current_speed_;

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received scan data. First range: %f, Frame: %s", 
                msg->ranges[0], msg->header.frame_id.c_str());
            double collision_threshold = 0.3;
            std::vector<double> ittc_values = compute_ittc(msg);
            RCLCPP_INFO(this->get_logger(), "ITT values: ");
            for (double ittc : ittc_values) {
                RCLCPP_INFO(this->get_logger(), "ITTL %f", ittc);
                if (ittc < collision_threshold) {
                    RCLCPP_INFO(this->get_logger(), "Detected collision %f", ittc);
                    apply_brake();
                    break;
                }
            }
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_speed_ = msg->twist.twist.linear.x;
            RCLCPP_INFO(this->get_logger(), "Car's velocity: %f", current_speed_);
        }

        std::vector<double> compute_ittc(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            std::vector<double> ittc_values;
                
            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                double range_val = msg->ranges[i];
                if (range_val <= 0 || range_val > msg->range_max) {
                    RCLCPP_WARN(this->get_logger(), "Invalid range value: %f", range_val);
                }    
                if (range_val > 0 && range_val < msg->range_max) {  // Ensure valid readings
                    double angle = msg->angle_min + i * msg->angle_increment;
                    double range_rate = current_speed_ * std::cos(angle);
                    if(range_rate > 0.01){
                        double ittc = range_val / range_rate;
                        ittc_values.push_back(ittc);
                    }
                }
            }
            return ittc_values;
        }

        void apply_brake() {
            RCLCPP_INFO(this->get_logger(), "Apply brakes. Speed: %f", current_speed_);
            
            auto brake_msg = ackermann_msgs::msg::AckermannDriveStamped();
            brake_msg.drive.speed = std::max(0.0, current_speed_ - 0.5);  // Stop the car
            brake_pub_->publish(brake_msg);
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EmergencyBraking>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}