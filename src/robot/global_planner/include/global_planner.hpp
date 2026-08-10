#ifndef GLOBAL_PLANNER_HPP_
#define GLOBAL_PLANNER_HPP_

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "global_planner/msg/reference_track.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fstream>
#include <memory>
#include <string>

class GlobalPlanner : public rclcpp::Node
{
public:
  GlobalPlanner();

private:
  void retrieveData(std::ifstream & file);

  global_planner::msg::ReferenceTrack reference_track_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<global_planner::msg::ReferenceTrack>::SharedPtr reference_pub_;
  std::string waypoint_frame_id_;
};

#endif
