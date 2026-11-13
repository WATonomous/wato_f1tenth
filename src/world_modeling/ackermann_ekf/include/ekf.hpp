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

#ifndef EKF_
#define EKF_

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <thread>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using vector3d = Eigen::Matrix<double, 3, 1>;
using vector4d = Eigen::Matrix<double, 4, 1>;
using vector5d = Eigen::Matrix<double, 5, 1>;
using vector7d = Eigen::Matrix<double, 7, 1>;
using matrix7d = Eigen::Matrix<double, 7, 7>;
using matrix5d = Eigen::Matrix<double, 5, 5>;
using matrix4d = Eigen::Matrix<double, 4, 4>;
using matrix3d = Eigen::Matrix<double, 3, 3>;
using matrix3x7d = Eigen::Matrix<double, 3, 7>;
using matrix7x3d = Eigen::Matrix<double, 7, 3>;
using matrix5x7d = Eigen::Matrix<double, 5, 7>;
using matrix4x7d = Eigen::Matrix<double, 4, 7>;
using matrix7x5d = Eigen::Matrix<double, 7, 5>;
using matrix7x4d = Eigen::Matrix<double, 7, 4>;

enum state_index
{
  x = 0,
  y = 1,
  theta = 2,
  v = 3,
  theta_dot = 4,
  ax = 5,
  ay = 6
};

struct prediction
{
  vector7d mu;
  matrix7d sigma_t;
  rclcpp::Time calc_time;
  bool did_error;
};

/*
the main goal of the ekf if to improve the accuracy of the speed and orientation (yaw) estimate
there is no need to try and correct or impove the model furthure, as making it more complex will
result in a higer computatinal cost
in the future, it might be possible to levrage something like compute shaders to
make a more sophictated model work, but that will end up being a seperate project entierly
*/
class EKF : public rclcpp::Node
{
public:
  EKF();

private:
  // publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_pub;

  // subscribers
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

  // helper functions
  void control_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr input);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  vector7d model_update(
    const vector7d & mu_prev, const ackermann_msgs::msg::AckermannDriveStamped & control_input, double dt_);
  matrix7d predict_sigma(const matrix7d & sigma_prev, const matrix7d & jacobian_g);
  matrix7d calc_jacobian_g(const vector7d & mu_prev, double dt_);
  prediction prediction_step(
    const vector7d & mu_prev,
    const matrix7d & sigma_prev,
    const ackermann_msgs::msg::AckermannDriveStamped & control_input);

  // IMU observation {theta_dot, a_forward, a_left} in base_link (the RealSense has no orientation)
  vector3d imu_state_mapper(const vector7d & mu_predicted);
  matrix3x7d imu_jacobian(const vector7d & mu_predicted);
  vector3d imu_observation_maker(const sensor_msgs::msg::Imu & observation);
  bool lookup_imu_rotation(const std::string & imu_frame);
  vector5d odom_state_mapper(const vector7d & mu_predicted);
  vector5d odom_observation_maker(const nav_msgs::msg::Odometry & observation);

  double velocity_lowPass(double current_speed);

  void init_time();
  void init_params();
  double calc_dt(rclcpp::Time & current_time, rclcpp::Time & prev_time);
  void publish_state();
  void publish_odom();
  void publish_tf();

  // data
  vector7d mu;
  matrix7d sigma_t;
  rclcpp::Time prev_update_time;
  ackermann_msgs::msg::AckermannDriveStamped prev_input;
  double prev_speed = 0.0;  // the speed low-pass reads this before any control input sets it

  // process noise {x , y , theta, v, theta_dot, ax , ay}
  matrix7d R;

  // sensor noise odom {x, y , theta , v, thata_dot}
  matrix5d Q_odom;

  // sensor noise imu {theta_dot, a_forward, a_left} (was {theta, theta_dot, ax, ay}: the RealSense has no yaw)
  matrix3d Q_imu;

  // H matrix for sensor model (the IMU one depends on theta, so it is built per update: imu_jacobian)
  matrix5x7d H_odom;  // 5 / 7 matrix

  // IMU frame -> base_link rotation, looked up once from TF (static: base_link -> camera_* frames)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  tf2::Matrix3x3 imu_rotation;
  std::string imu_rotation_frame;
  bool imu_rotation_ready = false;

  // publishing at a fixed rate: the callbacks run at ~300 Hz together, and the particle filter
  // runs one update per odometry message
  rclcpp::TimerBase::SharedPtr publish_timer;

  // IMU watchdog: warns when odometry arrives but the IMU doesn't
  rclcpp::TimerBase::SharedPtr imu_watchdog;
  rclcpp::Time last_imu_rx{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_rx{0, 0, RCL_ROS_TIME};
  double publish_rate_hz;
  bool state_valid = false;

  // Identity matrix
  matrix7d I7;

  // parameters
  std::string odom_topic, ekf_topic, imu_topic, ackermann_topic, child_frame, header_frame;
  double wheel_base;
  bool should_pub_tf;
  bool yaw_from_steering;
  double alpha;

  // tf2 broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // initalization
  bool time_init = false;
  bool state_init = false;

  // debug
  int error_counter = 0;
  int sucess_counter = 0;
  int sucess_counter_imu = 0;
  int sucess_counter_odom = 0;
  bool debug_mode;
};

#endif
