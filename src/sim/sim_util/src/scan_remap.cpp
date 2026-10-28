// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SCAN_REMAP : public rclcpp::Node
{
public:
  SCAN_REMAP()
  : Node("scan_remap")
  {
    // AutoDRIVE lidar -> contract scan, restamped into our laser frame
    this->declare_parameter<std::string>("input_topic", "/autodrive/roboracer_1/lidar");
    this->declare_parameter<std::string>("output_topic", "/hal/scan");
    this->declare_parameter<std::string>("frame_id", "laser");
    frame_id = this->get_parameter("frame_id").as_string();

    // initalize pubs and subs
    scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(this->get_parameter("output_topic").as_string(), 10);

    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      this->get_parameter("input_topic").as_string(),
      10,
      std::bind(&SCAN_REMAP::laser_callback, this, std::placeholders::_1));
  }

private:
  // pubs and subs
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;

  std::string frame_id;

  // functions
  void laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    sensor_msgs::msg::LaserScan new_scan = *msg;
    new_scan.header.set__frame_id(frame_id);
    scan_pub->publish(new_scan);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SCAN_REMAP>());
  rclcpp::shutdown();
  return 0;
}
