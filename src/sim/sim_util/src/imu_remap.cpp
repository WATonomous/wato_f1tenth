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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// AutoDRIVE IMU -> contract IMU. AutoDRIVE stamps it "imu", a frame in its own
// world -> roboracer_1 TF tree, so restamp it into imu_link under our base_link.
class IMU_REMAP : public rclcpp::Node
{
public:
  IMU_REMAP()
  : Node("imu_remap")
  {
    this->declare_parameter<std::string>("input_topic", "/autodrive/roboracer_1/imu");
    this->declare_parameter<std::string>("output_topic", "/hal/imu");
    this->declare_parameter<std::string>("frame_id", "imu_link");
    frame_id = this->get_parameter("frame_id").as_string();

    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(this->get_parameter("output_topic").as_string(), 10);

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      this->get_parameter("input_topic").as_string(),
      10,
      std::bind(&IMU_REMAP::imu_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

  std::string frame_id;

  void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu new_imu = *msg;
    new_imu.header.set__frame_id(frame_id);
    imu_pub->publish(new_imu);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMU_REMAP>());
  rclcpp::shutdown();
  return 0;
}
