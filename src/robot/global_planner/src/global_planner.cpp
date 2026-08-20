#include "global_planner.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <sstream>

GlobalPlanner::GlobalPlanner()
: Node("global_planner_node")
{
  declare_parameter<std::string>("file_directory", "/assets/e7_fifth_big_reference.csv");
  declare_parameter<std::string>("path_topic", "/global_planner/path");
  declare_parameter<std::string>("reference_track_topic", "/global_planner/reference_track");
  declare_parameter<std::string>("waypoint_frame_id", "map");

  waypoint_frame_id_ = get_parameter("waypoint_frame_id").as_string();
  const auto qos = rclcpp::QoS(1).transient_local().reliable();
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    get_parameter("path_topic").as_string(), qos);
  reference_pub_ = create_publisher<global_planner::msg::ReferenceTrack>(
    get_parameter("reference_track_topic").as_string(), qos);

  const auto file_path = ament_index_cpp::get_package_share_directory("global_planner") +
    get_parameter("file_directory").as_string();
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_FATAL(get_logger(), "Could not open raceline reference: %s", file_path.c_str());
    rclcpp::shutdown();
    return;
  }

  reference_track_.header.frame_id = waypoint_frame_id_;
  reference_track_.header.stamp = now();
  reference_track_.path.header = reference_track_.header;
  retrieveData(file);

  RCLCPP_INFO(
    get_logger(), "Loaded %zu raceline waypoints with widths",
    reference_track_.path.poses.size());
  path_pub_->publish(reference_track_.path);
  reference_pub_->publish(reference_track_);
}

void GlobalPlanner::retrieveData(std::ifstream & file)
{
  std::string line;
  std::getline(file, line);  // x_m,y_m,v_mps,dr_m,dl_m
  while (std::getline(file, line)) {
    std::stringstream row(line);
    std::string x, y, velocity, right, left;
    std::getline(row, x, ',');
    std::getline(row, y, ',');
    std::getline(row, velocity, ',');
    std::getline(row, right, ',');
    std::getline(row, left, ',');

    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header = reference_track_.header;
    waypoint.pose.position.x = std::stod(x);
    waypoint.pose.position.y = std::stod(y);
    waypoint.pose.position.z = std::stod(velocity);
    reference_track_.path.poses.push_back(waypoint);

    global_planner::msg::TrackWidth width;
    width.right_m = std::stof(right);
    width.left_m = std::stof(left);
    reference_track_.widths.push_back(width);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}
