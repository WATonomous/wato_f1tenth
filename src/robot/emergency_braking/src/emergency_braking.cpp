#include <cmath>
#include <vector>
#include <limits>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"


using std::placeholders::_1;

class EmergencyBraking : public rclcpp::Node {

   public:
        EmergencyBraking() : Node("emergency_braking_node") {
            RCLCPP_INFO(this->get_logger(), " node started");
            // publisher and subscriber
            brake_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/autodrive/f1tenth_1/lidar", 10, std::bind(&EmergencyBraking::scan_callback, this, _1));        }

   private:
        // constants
        static constexpr double threshold = 10.0; 
        static constexpr bool brake_default = false;
        static constexpr int window_size = 21; // how many lidar beams to use in forward direction
        static constexpr double min_speed = 0.0; // smallest speed to consider for calculations

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_pub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

        // lidar scan callback
        void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
            // index (idx) of beam at angle 0 (directly forward)
            // static_cast to convert double to int
            int forward_idx = static_cast<int>(std::round((0.0 - scan_msg->angle_min) / scan_msg->angle_increment));
            int half_window = window_size / 2;

            // Store previous ranges for each beam in the window to estimate speed
            // initialize each element to 0.0
            // window_size is number of elements
            static std::vector<double> prev_ranges(window_size, 0.0);
            // Store previous timestamp to compute time difference
            static rclcpp::Time prev_time = scan_msg->header.stamp;
            rclcpp::Time curr_time = scan_msg->header.stamp;
            // dt: change in time
            double dt = (curr_time - prev_time).seconds();
            prev_time = curr_time;

            // Minimum TTC found in window
            // set the initial value to the maximum 
            double min_ttc = std::numeric_limits<double>::max(); 
            
            // Number of valid beams used
            // start at 0 since no beams were used yet
            int valid_count = 0; 

            // iterate through the window of beams aroinund the forward direction
            for (int i = -half_window; i <= half_window; ++i) {
                int idx = forward_idx + i;
                // exit the loop if the index is negative or too large
                if (idx < 0 || idx >= static_cast<int>(scan_msg->ranges.size())) {
                    RCLCPP_INFO(this->get_logger(), "Skipping idx %d: out of bounds", idx);
                    continue;
                }
                // get lidar distance measurement at index idx
                double range = scan_msg->ranges[idx];

                // emergency stop if any beam is very close
                if (range < 0.3) {
                    auto brake_msg = std_msgs::msg::Float32();
                    brake_msg.data = 0.0;
                    RCLCPP_WARN(this->get_logger(), "Emergency stop: object too close!");
                    this->brake_pub_->publish(brake_msg);
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "idx: %d, range: %f", idx, range);
                RCLCPP_INFO(this->get_logger(), "forward_idx: %d, ranges.size: %zu", forward_idx, scan_msg->ranges.size());
                // exit the loop if the index is invalid
                if (std::isnan(range)) {
                    RCLCPP_INFO(this->get_logger(), "Skipping idx %d: NaN", idx);
                    continue;
                }
                if (range > scan_msg->range_max) {
                    RCLCPP_INFO(this->get_logger(), "Skipping idx %d: range > range_max (%f > %f)", idx, range, scan_msg->range_max);
                    continue;
                }
                if (range < scan_msg->range_min) {
                    RCLCPP_INFO(this->get_logger(), "Skipping idx %d: range < range_min (%f < %f)", idx, range, scan_msg->range_min);
                    continue;
                }

                // Reset prev_ranges if a large jump is detected
                int prev_idx = i + half_window;
                if (std::abs(prev_ranges[prev_idx] - range) > 2.0) {
                    prev_ranges[prev_idx] = range;
                    continue;
                }

                // previous index
                if (prev_ranges[prev_idx] == 0.0) prev_ranges[prev_idx] = range;

                double speed = 0.0;
                // Estimate speed using distance and time
                if (dt > 0.0) {
                    speed = (prev_ranges[prev_idx] - range) / dt;
                }
                prev_ranges[prev_idx] = range;


                // exit loop if speed is an extreme value
                if (speed <= min_speed || speed > 10.0) continue;

                double ttc = range / speed;
                if (ttc < min_ttc) min_ttc = ttc;

                valid_count++;
                
            }

            // If no valid beams, skip braking logic
            if (valid_count == 0) {
                RCLCPP_INFO(this->get_logger(), "No valid beams for TTC calculation.");
                return;
            }

            bool brake = brake_default;
            RCLCPP_INFO(this->get_logger(), "min_ttc: %f", min_ttc);

            // Brake if the minimum TTC is below the threshold
            if (min_ttc < threshold) {
                brake = true;
            }
            if (brake) {
                auto brake_msg = std_msgs::msg::Float32();
                brake_msg.data = 0.0;
                RCLCPP_INFO(this->get_logger(), "stop car");
                this->brake_pub_->publish(brake_msg);
            } else {
                // temporary code for making the car go forward for testing
                auto throttle_msg = std_msgs::msg::Float32();
                throttle_msg.data = 1.0; 
                RCLCPP_INFO(this->get_logger(), "go forward");
                this->brake_pub_->publish(throttle_msg);
            }
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmergencyBraking>());
    rclcpp::shutdown();
    return 0;
}