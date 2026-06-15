#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

class ImuDriveNode : public rclcpp::Node
{
public:
  ImuDriveNode() : Node("imu_drive_node")
  {
    // Subscribe to IMU
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/autodrive/roboracer_1/imu", 10,
      std::bind(&ImuDriveNode::imu_callback, this, std::placeholders::_1));

    // Publisher for throttle
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/roboracer_1/throttle_command", 10);

    // Timer to publish throttle at 10Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ImuDriveNode::publish_throttle, this));

    RCLCPP_INFO(this->get_logger(), "IMU Drive Node started!");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "IMU - Linear Accel: x=%.2f, y=%.2f, z=%.2f",
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);
  }

  void publish_throttle()
  {
    auto msg = std_msgs::msg::Float32();
    msg.data = 0.1;  //throttle
    throttle_pub_->publish(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuDriveNode>());
  rclcpp::shutdown();
  return 0;
}