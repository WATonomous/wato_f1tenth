#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>

class ImuThrottle : public rclcpp::Node
{
public:
    ImuThrottle();

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    bool stopped;
    float throttle;
};

ImuThrottle::ImuThrottle()
    : Node("imu_throttle"), stopped(false), throttle(-1.0f)
{
    RCLCPP_INFO(this->get_logger(), "IMU Throttle node started");

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/autodrive/f1tenth_1/imu_throttle", 10);
    throttle_pub_ = create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/autodrive/f1tenth_1/imu", 10,
        std::bind(&ImuThrottle::imuCallback, this, std::placeholders::_1));

    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/autodrive/f1tenth_1/lidar", 10,
        std::bind(&ImuThrottle::lidarCallback, this, std::placeholders::_1));
}

void ImuThrottle::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received IMU: [%.3f, %.3f, %.3f]",
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);

    if (msg->angular_velocity.x > 0.1 ||
        msg->angular_velocity.y > 0.1 ||
        msg->angular_velocity.z > 0.1)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing filtered IMU");
        imu_pub_->publish(*msg);
    }
}

void ImuThrottle::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (msg->ranges.empty())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "empty ranges from LiDAR");
        return;
    }

    int center_index = msg->ranges.size() / 2;
    RCLCPP_INFO(this->get_logger(), "Front distance: %.2f", msg->ranges[center_index]);
    float front_distance = msg->ranges[center_index];

    if (std::isnan(front_distance) ||
        front_distance < msg->range_min)
    {
        return;
    }

    constexpr float STOP_THRESHOLD = 2.0f;

    float desired_throttle;

    if (front_distance < STOP_THRESHOLD)
    {
        desired_throttle = 0.0f;
        if (desired_throttle != throttle)
        {
            std_msgs::msg::Float32 throttle_msg;
            throttle_msg.data = desired_throttle;
            throttle_pub_->publish(throttle_msg);
            throttle = desired_throttle;
            RCLCPP_INFO(this->get_logger(), "Obstacle at %.2fm — setting throttle to 0.0", front_distance);
        }
        return;
    }
    desired_throttle = 1.0f;

    RCLCPP_INFO(this->get_logger(), "Setting throttle to 1.0f");

    if (desired_throttle != throttle)
    {
        std_msgs::msg::Float32 throttle_msg;
        throttle_msg.data = desired_throttle;
        throttle_pub_->publish(throttle_msg);
        throttle = desired_throttle;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuThrottle>());
    rclcpp::shutdown();
    return 0;
}