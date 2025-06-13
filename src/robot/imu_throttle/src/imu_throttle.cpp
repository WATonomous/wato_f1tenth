#include "imu_throttle.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"        // Need Float32, not Int32
#include "std_msgs/msg/int32.hpp"          // Keep Int32 if you want to use it

using namespace std::chrono_literals;

class ImuThrottle : public rclcpp::Node
{
public:
    ImuThrottle() : Node("imu_throttle")
    {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autodrive/f1tenth_1/imu", 10,
            std::bind(&ImuThrottle::imu_callback, this, std::placeholders::_1));

        // FIXED: Publisher type now matches the template parameter
        throttle_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/throttle_command", 10);

        throttle_timer_ = this->create_wall_timer(
            100ms, std::bind(&ImuThrottle::throttle_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "started IMU throttle node");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_publisher_;  // FIXED: Now Float32
    rclcpp::TimerBase::SharedPtr throttle_timer_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double threshold = 0.5;
        double lin_acc_x = msg->linear_acceleration.x;

        if (std::abs(lin_acc_x) > threshold) {
            // FIXED: Added throttling to prevent console spam
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "linear accel: %.3f m/s²", lin_acc_x);
        }
    }

    void throttle_timer_callback()
    {
        // FIXED: Now using Float32 message type
        auto throttle_msg = std_msgs::msg::Float32();

        throttle_msg.data = 1.0f;  // FIXED: Float value, not int

        throttle_publisher_->publish(throttle_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "published throttle data: %.2f", throttle_msg.data);  // FIXED: %.2f for float
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuThrottle>());
    rclcpp::shutdown();
    return 0;
}