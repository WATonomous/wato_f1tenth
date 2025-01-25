#include "pure_pursuit.hpp"

class PurePursuit : public rclcpp::Node {
public:
  PurePursuit(): Node("pure_pursuit") {
    // Publishers
    drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

    // Subscribers
    point_subscriber = this->create_subscription<geometry_msgs::msg::Point>("/planning_point", 10,
    std::bind(&PurePursuit::goal_subscriber_callback, this, std::placeholders::_1));

    // Publish timers
    publisher_timer_ = this->create_wall_timer(15ms, std::bind(&PurePursuit::publisher_timer_callback, this));

    // Some initial values so it won't error upon first publish
    int pub_drive_speed = 0;
    int pub_steering_angle = 0;

  }

private:
  // Pub, Sub & Timers
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr gym_drive_publisher;
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  rclcpp::Subscriber<geometry_msgs::msg::Point>::SharedPtr point_subscriber;

  void publisher_timer_callback();
  void goal_subscriber_callback();
  void find_drive_params();
};

void PurePursuit::publisher_timer_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());

  // SAMPLE CODE TO MOVE THE ROBOT STRAIGHT
  auto straight_drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

  // Compute these values inside pure pursuit algorithm
  straight_drive_msg.drive.speed = 0.3;
  straight_drive_msg.drive.steering_angle = 0.0; 

  // Publish
  drive_publisher_->publish(straight_drive_msg);
}

void PurePursuit::goal_subscriber_callback() {
    find_drive_params();
}

void PurePursuit::find_drive_params() {
    return;
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuit>());
  rclcpp::shutdown();
  return 0;
}