// simplified program to test publishers and subscribers before testing logic

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
//#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <vector>
#include <cmath>

class TestDrive : public rclcpp::Node {
    public:
    // constructor
        TestDrive() : Node("test_drive") {
            drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&TestDrive::scan_callback, this, std::placeholders::_1)
            )
            RCLCPP_INFO(this->get_logger(), "node started");
        }
    private:
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;  // Added subscription to /scan
        double current_speed_;
        // call when messages are received
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            if (msg->ranges[0] < 1.0) {  
                apply_brake();
            }
        }
        void apply_brake(){
            RCLCPP_INFO(this->get_logger(), "apply brake");
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.speed = 0.0;
            drive_pub->publish(drive_msg);
        }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestDrive>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}