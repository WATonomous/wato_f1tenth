// publish fake scan data to simulate oncoming obstacles

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>

class ObstaclePublisher : public rclcpp::Node {
    public:
        ObstaclePublisher() : Node("obstacle_publisher") {
            scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), std::bind(&ObstaclePublisher::publish_obstacle, this)
            );
        }
    private:
        void publish_obstacle(){

            auto msg = sensor_msgs::msg::LaserScan();
            msg.header.stamp = this->now();
            msg.header.frame_id = "ego_racecar/laser";
            msg.angle_min = -1.57;  // -90 degrees
            msg.angle_max = 1.57;   // +90 degrees
            msg.angle_increment = 0.017;  // 1 degree per step
            msg.range_min = 0.2;
            msg.range_max = 10.0;

            size_t num_readings = (msg.angle_max - msg.angle_min) / msg.angle_increment;
            msg.ranges.resize(num_readings, 10.0);  // Default: no obstacles (max range)

            // Simulate an obstacle at 1.0m in front
            int center_index = num_readings / 2;
            msg.ranges[center_index] = 1.0;

            scan_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published fake scan data.");
        }

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
        rclcpp::TimerBase::SharedPtr timer_;      
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstaclePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
