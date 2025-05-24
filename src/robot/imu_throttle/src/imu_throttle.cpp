#include "imu_throttle.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"

using namespace std::chrono_literals;
class ImuThrottle : public rclcpp::Node
{
public:
    ImuThrottle() : Node("imu_throttle")
    {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autodrive/f1tenth_1/imu", 10,
            std::bind(&ImuThrottle::imu_callback, this, std::placeholders::_1));

        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
            "/autodrive/f1tenth_1/ackermann_cmd", 10);

        throttle_timer_ = this->create_wall_timer(
            100ms, std::bind(&ImuThrottle::throttle_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "started IMU node");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_publisher_;
    rclcpp::TimerBase::SharedPtr throttle_timer_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double threshold = 0.5;
        double lin_acc_x = msg->linear_acceleration.x;

        if (std::abs(lin_acc_x) > threshold) {
            RCLCPP_INFO(this->get_logger(),
                        "linear accel: %.3f m/s²", lin_acc_x);
        }
    }

    void throttle_timer_callback()
    {
        auto drive_msg = ackermann_msgs::msg::AckermannDrive();

        drive_msg.speed = 0.5;          // constant throttle speed
        drive_msg.steering_angle = 0.0; // no steering, straight

        drive_publisher_->publish(drive_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "send speed %.2f", drive_msg.speed);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuThrottle>());
    rclcpp::shutdown();
    return 0;
}