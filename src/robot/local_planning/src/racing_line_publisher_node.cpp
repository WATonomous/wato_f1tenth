#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace local_planning
{
class RacingLinePublisherNode : public rclcpp::Node
{
public:
  RacingLinePublisherNode() : Node("racing_line_publisher_node")
  {
    declare_parameter<std::string>("racing_line_file", "");
    declare_parameter<std::string>("racing_line_topic", "/racing_line");
    declare_parameter<std::string>("waypoint_frame_id", "map");
    const auto file_name = get_parameter("racing_line_file").as_string();
    path_.header.frame_id = get_parameter("waypoint_frame_id").as_string();
    publisher_ = create_publisher<nav_msgs::msg::Path>(get_parameter("racing_line_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    std::ifstream file(file_name);
    if (!file.is_open()) {throw std::runtime_error("Could not open racing-line CSV: " + file_name);}
    std::string line;
    std::getline(file, line);
    while (std::getline(file, line)) {
      std::stringstream row(line); std::string x, y, velocity;
      if (!std::getline(row, x, ',') || !std::getline(row, y, ',') || !std::getline(row, velocity, ',')) {continue;}
      try {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = path_.header.frame_id;
        pose.pose.position.x = std::stod(x); pose.pose.position.y = std::stod(y); pose.pose.position.z = std::stod(velocity);
        path_.poses.push_back(std::move(pose));
      } catch (const std::exception &) {RCLCPP_WARN(get_logger(), "Skipping invalid CSV row: %s", line.c_str());}
    }
    if (path_.poses.empty()) {throw std::runtime_error("Racing-line CSV contains no valid waypoints");}
    publish();
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {publish();});
  }
private:
  void publish() {path_.header.stamp = now(); publisher_->publish(path_);}
  nav_msgs::msg::Path path_; rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_; rclcpp::TimerBase::SharedPtr timer_;
};

class BoolTopicPublisherNode : public rclcpp::Node
{
public:
  BoolTopicPublisherNode() : Node("bool_topic_publisher_node")
  {
    declare_parameter<std::string>("topic", "/overtake_ready"); declare_parameter<bool>("value", true); declare_parameter<double>("publish_rate_hz", 1.0);
    value_.data = get_parameter("value").as_bool();
    publisher_ = create_publisher<std_msgs::msg::Bool>(get_parameter("topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    const auto rate = std::max(0.1, get_parameter("publish_rate_hz").as_double());
    publish(); timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] {publish();});
  }
private:
  void publish() {publisher_->publish(value_);}
  std_msgs::msg::Bool value_; rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_; rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace local_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  if (std::string(argv[0]).find("bool_topic_publisher") != std::string::npos) {
    rclcpp::spin(std::make_shared<local_planning::BoolTopicPublisherNode>());
  } else {rclcpp::spin(std::make_shared<local_planning::RacingLinePublisherNode>());}
  rclcpp::shutdown(); return 0;
}
