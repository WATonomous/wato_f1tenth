#include "pure_pursuit.hpp"

class PurePursuit : public rclcpp::Node {
public:
  PurePursuit(): Node("pure_pursuit"), count_(0) {
    // Publishers
    drive_publisher = this->create_publisher<ackermann_msgs/msg/ackermann_drive_stamped>("drive", 10);

    // Subscribers
    point_subscriber = this->create_subscriber<geometry_msgs/msg/point>("/planning_point", 10,
    std::bind(&PurePursuit::goal_subscriber_callback, this, std::placeholders::_1));

    // Publish timers
    publisher_timer_ = this->create_wall_timer(15ms, std::bind(&PurePursuit::publisher_timer_callback, this));

    // Some initial values so it won't error upon first publish
    int pub_drive_speed = 0;
    int pub_steering_angle = 0;

}

private:
  // Pub, Sub & Timers
  rclcpp::Publisher<ackermann_msgs/msg/ackermann_drive_stamped>::SharedPtr gym_drive_publisher;
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  rclcpp::Subscriber<geometry_msgs/msg/point>::SharedPtr point_subscriber;

  // Next driving command to publish
  int pub_drive_speed;
  int pub_steering_angle;

  void publisher_timer_callback();
  void goal_subscriber_callback();
  void find_drive_params();
}

void PurePursuit::publisher_timer_callback(const geometry_msgs::msg::point::SharedPtr msg) {
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
    pub_drive_speed = 0.3;
    pub_steering_angle = 0.3;
    return;
}