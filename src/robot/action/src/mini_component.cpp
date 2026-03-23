#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"

class MiniComponent : public rclcpp::Node
{
public:
    MiniComponent() : Node("mini_component")
    {
        RCLCPP_INFO(this->get_logger(), "Mini Component Node Started");

        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autodrive/f1tenth_1/imu",
            rclcpp::SensorDataQoS(),
            std::bind(&MiniComponent::imu_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/throttle_command", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MiniComponent::timer_callback, this));
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "imu aceel x: %f", msg->linear_acceleration.x);
    }

    void timer_callback()
    {
        auto message = std_msgs::msg::Float32();
        message.data = 0.2;
        pub_->publish(message);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiniComponent>());
    rclcpp::shutdown();
    return 0;
}