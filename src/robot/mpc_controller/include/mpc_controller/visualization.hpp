#pragma once
#include "mpc_controller/controller.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace mpc::visualization {
using Markers = visualization_msgs::msg::MarkerArray;

Markers track(const Reference & reference, const std_msgs::msg::Header & header);
nav_msgs::msg::OccupancyGrid searchSpace(const Scene & scene, const std_msgs::msg::Header & header);
nav_msgs::msg::OccupancyGrid emptySearchSpace(const std_msgs::msg::Header & header);
nav_msgs::msg::Path path(const Solution & solution, const std_msgs::msg::Header & header);
Markers cycle(const Cycle * result, const std::vector<Pose> & route,
  Pose ego, bool state_valid, bool approved, const std::string & reason,
  const std_msgs::msg::Header & header);
}  // namespace mpc::visualization
