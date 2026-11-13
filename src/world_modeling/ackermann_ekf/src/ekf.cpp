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

#include "ekf.hpp"  // NOLINT(build/include_subdir)

#include <memory>
#include <string>

EKF::EKF()
: Node("ekf")
{
  EKF::init_params();

  // subs
  control_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    ackermann_topic, 10, std::bind(&EKF::control_callback, this, std::placeholders::_1));

  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&EKF::odom_callback, this, std::placeholders::_1));

  // Best effort: the RealSense publishes the IMU best effort, and a reliable subscription to a
  // best-effort publisher gets nothing at all ("offering incompatible QoS"), which is how the EKF ran
  // without the IMU on the car on 2026-11-12. Best effort also receives from reliable publishers.
  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(), std::bind(&EKF::imu_callback, this, std::placeholders::_1));

  // pubs
  ekf_pub = this->create_publisher<nav_msgs::msg::Odometry>(ekf_topic, 10);

  RCLCPP_INFO(this->get_logger(), "initalized pubs and subs");

  // tf broadcast
  if (should_pub_tf) {
    RCLCPP_INFO(this->get_logger(), "tf is enabled");
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  // tf listener: the IMU frame -> base_link rotation (the RealSense IMU is in camera_imu_optical_frame)
  tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // warn while odometry arrives but the IMU doesn't: the EKF then runs on wheel odometry alone, whose
  // yaw rate comes from the steering command
  imu_watchdog = this->create_wall_timer(std::chrono::seconds(1), [this]() {
    const rclcpp::Time now = this->now();
    const bool odom_live = last_odom_rx.nanoseconds() != 0 && (now - last_odom_rx).seconds() < 1.0;
    const bool imu_live = last_imu_rx.nanoseconds() != 0 && (now - last_imu_rx).seconds() < 1.0;
    if (odom_live && !imu_live) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "no IMU on %s (odometry is arriving): heading comes from the wheel odometry only", imu_topic.c_str());
    }
  });

  // publish at a fixed rate instead of from every callback (publish_rate_hz <= 0: every update, as before)
  if (publish_rate_hz > 0.0) {
    publish_timer = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz), [this]() {
        if (state_valid) {
          EKF::publish_state();
        }
      });
  }
}

void EKF::control_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr input)
{
  // The odometry and IMU callbacks start the clock too now: the jacobian corrupting without a control
  // input was most likely the uninitialized matrices (.Zero()), and prev_input defaults to a stop.
  if (!time_init && input != nullptr) {
    EKF::init_time();
    prev_input = *input;
    prev_speed = input->drive.speed;
    RCLCPP_INFO(this->get_logger(), "initalized time and prev control input");
    return;
  }

  if (input == nullptr) {
    return;
  }

  prediction current_predition = EKF::prediction_step(mu, sigma_t, *input);

  prev_input = *input;  // save this regardless, a fail. always want the latest control input
  if (current_predition.did_error) {
    // RCLCPP_INFO(this->get_logger(), "control_callback predition corrupt");
    return;
  }

  // save the previous state
  prev_update_time = current_predition.calc_time;
  mu = current_predition.mu;
  sigma_t = current_predition.sigma_t;

  state_valid = true;
  if (publish_rate_hz <= 0.0) {
    EKF::publish_state();
  }
}

/*
The RealSense IMU has no orientation (only gyro and accelerometer), so the IMU no longer measures
yaw directly. It measures, in base_link:
  - the yaw rate (gyro about base_link z), which the model integrates into theta
  - the forward and left acceleration, which are the world-frame ax, ay rotated by theta
The message's frame (camera_imu_optical_frame on the car) is rotated into base_link with TF first.
*/
void EKF::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  if (imu_msg == nullptr) {
    return;
  }
  last_imu_rx = this->now();
  if (!time_init) {
    // start on any input: on the car /hal/ackermann_cmd is silent until something drives
    EKF::init_time();
    return;
  }

  if (!EKF::lookup_imu_rotation(imu_msg->header.frame_id)) {
    return;
  }

  prediction current_predition = EKF::prediction_step(mu, sigma_t, prev_input);
  if (current_predition.did_error) {
    return;
  }

  // the accelerations depend on theta, so H is the jacobian of the measurement function at the prediction
  matrix3x7d H_imu = EKF::imu_jacobian(current_predition.mu);

  // calculate kalman gain
  matrix3d S = (H_imu * current_predition.sigma_t * H_imu.transpose() + Q_imu).eval();  // inovation covariance
  S = (0.5 * (S + S.transpose())).eval();  // correct the matrix scew

  // note to future self, generalize the kalman gain calculation with a template function and kalman gain class

  // use the llt to try and solve
  Eigen::LLT<matrix3d> llt(S);
  matrix7x3d K;

  if (llt.info() != Eigen::Success) {
    // given llt fails, matrix is not positive definite, add jitter to fix
    // try using ldlt
    S.diagonal().array() += 1e-8;
    S = (0.5 * (S + S.transpose())).eval();

    Eigen::LDLT<matrix3d> ldlt(S);
    RCLCPP_INFO(this->get_logger(), "using ldlt to solve inversion in imu");

    if (ldlt.info() != Eigen::Success) {
      RCLCPP_INFO(this->get_logger(), "faild to safely invert S, discarding this iteration of imu correction");
      return;

    } else {
      K = (current_predition.sigma_t * H_imu.transpose() * ldlt.solve(matrix3d::Identity())).eval();
    }

  } else {
    K = (current_predition.sigma_t * H_imu.transpose() * llt.solve(matrix3d::Identity())).eval();
  }

  vector3d imu_diff = (imu_observation_maker(*imu_msg) - imu_state_mapper(current_predition.mu)).eval();
  vector7d new_mu = (current_predition.mu + K * imu_diff).eval();

  // josephs formula for new covariance (sigma_t)
  matrix7d A = (matrix7d::Identity() - K * H_imu).eval();
  matrix7d new_sigma_t = (A * current_predition.sigma_t * A.transpose() + K * Q_imu * K.transpose()).eval();

  // final safety check
  if (new_sigma_t.diagonal().minCoeff() <= 0) {
    new_sigma_t.diagonal().array() += 1e-8;
    new_sigma_t = (0.5 * (new_sigma_t + new_sigma_t.transpose())).eval();
    RCLCPP_INFO(this->get_logger(), "atempting to settle the covariance matrix in imu call back");
  }

  if (!new_mu.allFinite() || !new_sigma_t.allFinite()) {
    if (!new_mu.allFinite())
      RCLCPP_INFO(this->get_logger(), "mu corrupted during imu correction step, discarding results");

    if (!new_sigma_t.allFinite())
      RCLCPP_INFO(this->get_logger(), "simga_t corrupted during imu correction step, discarding results");

    return;
  }

  // normalize the yaw
  new_mu(state_index::theta) = std::atan2(std::sin(new_mu(state_index::theta)), std::cos(new_mu(state_index::theta)));

  // save the results
  mu = new_mu;
  sigma_t = new_sigma_t;
  prev_update_time = current_predition.calc_time;

  // publish the results (on the timer, or here when publish_rate_hz <= 0)
  state_valid = true;
  if (publish_rate_hz <= 0.0) {
    EKF::publish_state();
  }
}

void EKF::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  if (odom_msg == nullptr) {
    return;
  }
  last_odom_rx = this->now();
  if (!time_init) {
    // start on any input: on the car /hal/ackermann_cmd is silent until something drives
    EKF::init_time();
    return;
  }

  prediction current_predition = EKF::prediction_step(mu, sigma_t, prev_input);
  if (current_predition.did_error) {
    return;
  }

  // calculate kalman gain
  matrix5d S = (H_odom * current_predition.sigma_t * H_odom.transpose() + Q_odom).eval();  // inovation covariance
  S = (0.5 * (S + S.transpose())).eval();  // correct the matrix scew

  // use the llt to try and solve
  Eigen::LLT<matrix5d> llt(S);
  matrix7x5d K;

  if (llt.info() != Eigen::Success) {
    // given llt fails, matrix is not positive definite, add jitter to fix
    // try using ldlt
    S.diagonal().array() += 1e-8;
    S = (0.5 * (S + S.transpose())).eval();

    Eigen::LDLT<matrix5d> ldlt(S);
    RCLCPP_INFO(this->get_logger(), "using ldlt to solve inversion in odom");

    if (ldlt.info() != Eigen::Success) {
      RCLCPP_INFO(this->get_logger(), "faild to safely invert S, discarding this iteration of odom correction");
      return;

    } else {
      K = (current_predition.sigma_t * H_odom.transpose() * ldlt.solve(matrix5d::Identity())).eval();
    }

  } else {
    K = (current_predition.sigma_t * H_odom.transpose() * llt.solve(matrix5d::Identity())).eval();
  }

  vector5d odom_diff = (odom_observation_maker(*odom_msg) - odom_state_mapper(current_predition.mu)).eval();
  vector7d new_mu = (current_predition.mu + K * odom_diff).eval();

  // josephs formula for new covariance (sigma_t)
  matrix7d A = (matrix7d::Identity() - K * H_odom).eval();
  matrix7d new_sigma_t = (A * current_predition.sigma_t * A.transpose() + K * Q_odom * K.transpose()).eval();

  // final safety check
  if (new_sigma_t.diagonal().minCoeff() <= 0) {
    new_sigma_t.diagonal().array() += 1e-8;
    new_sigma_t = (0.5 * (new_sigma_t + new_sigma_t.transpose())).eval();
    RCLCPP_INFO(this->get_logger(), "atempting to settle the covariance matrix in odom call back");
  }

  if (!new_mu.allFinite() || !new_sigma_t.allFinite()) {
    if (!new_mu.allFinite())
      RCLCPP_INFO(this->get_logger(), "mu corrupted during odom correction step, discarding results");

    if (!new_sigma_t.allFinite())
      RCLCPP_INFO(this->get_logger(), "simga_t corrupted during odom correction step, discarding results");

    return;
  }

  // normalize the yaw
  new_mu(state_index::theta) = std::atan2(std::sin(new_mu(state_index::theta)), std::cos(new_mu(state_index::theta)));

  // save the results
  mu = new_mu;
  sigma_t = new_sigma_t;
  prev_update_time = current_predition.calc_time;

  // publish the results (on the timer, or here when publish_rate_hz <= 0)
  state_valid = true;
  if (publish_rate_hz <= 0.0) {
    EKF::publish_state();
  }
}

/*
Motion model, state {x, y, theta, v, theta_dot, ax, ay} (ax, ay are world-frame accelerations).

Position follows the state speed v, which the wheel odometry corrects. It used to follow the
commanded speed, which differs from the real one while the speed limiter ramps, or when the car is
blocked. The jacobian already assumed the state speed.

Yaw (yaw_from_steering false, the default): theta integrates the theta_dot state, and theta_dot is
held between updates. The IMU gyro (and, loosely, the wheel odometry) corrects theta_dot, so the
gyro now steers the heading. Before, theta and theta_dot were rebuilt from the steering command every
prediction, which overwrote any gyro correction.
Yaw (yaw_from_steering true): the previous bicycle model, theta_dot = v / L * tan(steering).
*/
vector7d EKF::model_update(
  const vector7d & mu_prev, const ackermann_msgs::msg::AckermannDriveStamped & control_input, double dt_)
{
  vector7d predicted_state;
  const double v = mu_prev(state_index::v);
  const double theta = mu_prev(state_index::theta);
  const double ax = mu_prev(state_index::ax);
  const double ay = mu_prev(state_index::ay);

  double theta_dot = mu_prev(state_index::theta_dot);
  if (yaw_from_steering) {
    theta_dot = (v / wheel_base) * std::tan(control_input.drive.steering_angle);
  }
  const double theta_new = theta + theta_dot * dt_;

  predicted_state << mu_prev(state_index::x) + v * std::cos(theta) * dt_ + 0.5 * ax * std::pow(dt_, 2),
    mu_prev(state_index::y) + v * std::sin(theta) * dt_ + 0.5 * ay * std::pow(dt_, 2),
    std::atan2(std::sin(theta_new), std::cos(theta_new)),  // wrapped to [-pi, pi]
    v + ax * std::cos(theta) * dt_ + ay * std::sin(theta) * dt_, theta_dot, ax, ay;

  return predicted_state.eval();
}

matrix7d EKF::predict_sigma(const matrix7d & sigma_prev, const matrix7d & jacobian_g)
{
  matrix7d jacobian_g_transpose = jacobian_g.transpose().eval();
  matrix7d sigma_predicted = (jacobian_g * sigma_prev * jacobian_g_transpose + R).eval();

  if (debug_mode) {
    float max_coeff = sigma_prev.diagonal().maxCoeff();
    float min_coeff = sigma_prev.diagonal().minCoeff();
    RCLCPP_INFO(this->get_logger(), " pre predicton current max coef is %f and min coeff is %f", max_coeff, min_coeff);
    RCLCPP_INFO(this->get_logger(), "the largest G coeff is %f", jacobian_g.maxCoeff());

    max_coeff = sigma_predicted.diagonal().maxCoeff();
    min_coeff = sigma_predicted.diagonal().minCoeff();
    RCLCPP_INFO(this->get_logger(), " post predicton current max coef is %f and min coeff is %f", max_coeff, min_coeff);

    RCLCPP_INFO(this->get_logger(), "jacobian G");
    for (int i = 0; i < 7; i++) {
      RCLCPP_INFO(
        this->get_logger(),
        " %f %f %f %f %f %f %f",
        jacobian_g(i, 0),
        jacobian_g(i, 1),
        jacobian_g(i, 2),
        jacobian_g(i, 3),
        jacobian_g(i, 4),
        jacobian_g(i, 5),
        jacobian_g(i, 6));
    }
  }

  return sigma_predicted.eval();
}

matrix7d EKF::calc_jacobian_g(const vector7d & mu_prev, double dt_)
{
  matrix7d jacobian;
  jacobian.setZero();  // was jacobian.Zero(), which returns a zero matrix and leaves this one uninitialized

  double DT = dt_, v = mu_prev(state_index::v), theta = mu_prev(state_index::theta),
         phi = prev_input.drive.steering_angle;
  double L_WB = wheel_base, ax = mu_prev(state_index::ax), ay = mu_prev(state_index::ay);

  if (debug_mode) {
    RCLCPP_INFO(
      this->get_logger(),
      "dt is %f, v is %f, theta is %f, phi is %f, wheel_base is %f, ax is %f, ay is %f",
      DT,
      v,
      theta,
      phi,
      L_WB,
      ax,
      ay);
  }

  // 1st row: x
  jacobian(0, 0) = 1.0;
  jacobian(0, 2) = -v * std::sin(theta) * DT;
  jacobian(0, 3) = std::cos(theta) * DT;
  jacobian(0, 5) = 0.5 * std::pow(DT, 2);

  // 2nd row: y
  jacobian(1, 1) = 1.0;
  jacobian(1, 2) = v * std::cos(theta) * DT;
  jacobian(1, 3) = std::sin(theta) * DT;
  jacobian(1, 6) = 0.5 * std::pow(DT, 2);

  // 3rd row: theta
  jacobian(2, 2) = 1.0;
  if (yaw_from_steering) {
    jacobian(2, 3) = (std::tan(phi) * DT) / L_WB;
  } else {
    jacobian(2, 4) = DT;
  }

  // 4th row: v
  jacobian(3, 2) = ay * std::cos(theta) * DT - ax * std::sin(theta) * DT;
  jacobian(3, 3) = 1.0;
  jacobian(3, 5) = std::cos(theta) * DT;
  jacobian(3, 6) = std::sin(theta) * DT;

  // 5th row: theta_dot
  if (yaw_from_steering) {
    jacobian(4, 3) = std::tan(phi) / L_WB;
  } else {
    jacobian(4, 4) = 1.0;
  }

  // 6th row: ax
  jacobian(5, 5) = 1.0;

  // 7th row: ay
  jacobian(6, 6) = 1.0;

  return jacobian.eval();
}

prediction EKF::prediction_step(
  const vector7d & mu_prev,
  const matrix7d & sigma_prev,
  const ackermann_msgs::msg::AckermannDriveStamped & control_input)
{
  prediction current_prediction;

  rclcpp::Time current_time = this->now();
  double dt = EKF::calc_dt(current_time, prev_update_time);

  // This print was once needed to keep the filter working ("some weird timing bug"). The likely cause
  // was the matrices that .Zero() / .Identity() left uninitialized (now setZero / setIdentity). It is
  // throttled now: unthrottled it ran at every prediction, about 300 times a second.
  if (error_counter == 0) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "current dt is %f", dt);
  }

  if (dt <= 1e-6) {
    current_prediction.did_error = true;
    RCLCPP_DEBUG(this->get_logger(), "dt is too small");
    return current_prediction;
  }

  vector7d mu_bar = EKF::model_update(mu_prev, control_input, dt).eval();
  matrix7d jacobian_G = EKF::calc_jacobian_g(mu_prev, dt).eval();
  matrix7d sigma_bar = EKF::predict_sigma(sigma_prev, jacobian_G).eval();

  // normalize the yaw
  mu_bar(state_index::theta) = std::atan2(std::sin(mu_bar(state_index::theta)), std::cos(mu_bar(state_index::theta)));

  if (!mu_bar.allFinite() || !sigma_bar.allFinite()) {
    current_prediction.did_error = true;
    if (!mu_bar.allFinite()) {
      RCLCPP_INFO(this->get_logger(), "mu_bar prediction is corrupt");
    }
    if (!sigma_bar.allFinite() && error_counter <= 4) {
      RCLCPP_INFO(this->get_logger(), "sigma_bar prediction is corrupt");
      error_counter++;
    }
  } else {
    current_prediction.did_error = false;
    current_prediction.mu = mu_bar.eval();
    current_prediction.sigma_t = sigma_bar.eval();
    current_prediction.calc_time = current_time;
  }

  return current_prediction;
}

// what the IMU should read for a state: yaw rate, and the world-frame ax, ay rotated into the car
vector3d EKF::imu_state_mapper(const vector7d & mu_predicted)
{
  vector3d observation;
  const double theta = mu_predicted(state_index::theta);
  const double ax = mu_predicted(state_index::ax);
  const double ay = mu_predicted(state_index::ay);

  observation(0) = mu_predicted(state_index::theta_dot);
  observation(1) = ax * std::cos(theta) + ay * std::sin(theta);  // forward
  observation(2) = -ax * std::sin(theta) + ay * std::cos(theta);  // left

  return observation.eval();
}

matrix3x7d EKF::imu_jacobian(const vector7d & mu_predicted)
{
  matrix3x7d H;
  H.setZero();
  const double theta = mu_predicted(state_index::theta);
  const double ax = mu_predicted(state_index::ax);
  const double ay = mu_predicted(state_index::ay);

  H(0, state_index::theta_dot) = 1.0;

  H(1, state_index::theta) = -ax * std::sin(theta) + ay * std::cos(theta);
  H(1, state_index::ax) = std::cos(theta);
  H(1, state_index::ay) = std::sin(theta);

  H(2, state_index::theta) = -ax * std::cos(theta) - ay * std::sin(theta);
  H(2, state_index::ax) = -std::sin(theta);
  H(2, state_index::ay) = std::cos(theta);

  return H.eval();
}

vector5d EKF::odom_state_mapper(const vector7d & mu_predicted)
{
  vector5d observation;
  observation.setZero();

  observation(0) = mu_predicted(state_index::x);
  observation(1) = mu_predicted(state_index::y);
  observation(2) = mu_predicted(state_index::theta);
  observation(3) = mu_predicted(state_index::v);
  observation(4) = mu_predicted(state_index::theta_dot);

  return observation.eval();
}

// the IMU reading in base_link: {yaw rate, forward acceleration, left acceleration}
vector3d EKF::imu_observation_maker(const sensor_msgs::msg::Imu & observation)
{
  vector3d z;

  const tf2::Vector3 gyro = imu_rotation * tf2::Vector3(
                                             observation.angular_velocity.x, observation.angular_velocity.y,
                                             observation.angular_velocity.z);
  const tf2::Vector3 accel = imu_rotation * tf2::Vector3(
                                              observation.linear_acceleration.x, observation.linear_acceleration.y,
                                              observation.linear_acceleration.z);

  z(0) = gyro.z();
  z(1) = accel.x();
  z(2) = accel.y();

  return z.eval();
}

// rotation from the IMU message's frame into base_link (child_frame), from TF; true once known
bool EKF::lookup_imu_rotation(const std::string & imu_frame)
{
  if (imu_rotation_ready && imu_frame == imu_rotation_frame) {
    return true;
  }

  if (imu_frame.empty() || imu_frame == child_frame) {
    imu_rotation.setIdentity();
  } else {
    try {
      const geometry_msgs::msg::TransformStamped t =
        tf_buffer->lookupTransform(child_frame, imu_frame, tf2::TimePointZero);
      imu_rotation.setRotation(tf2::Quaternion(
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "no TF %s -> %s yet, skipping IMU: %s",
        imu_frame.c_str(), child_frame.c_str(), ex.what());
      return false;
    }
  }

  imu_rotation_frame = imu_frame;
  imu_rotation_ready = true;
  RCLCPP_INFO(this->get_logger(), "IMU frame %s rotated into %s", imu_frame.c_str(), child_frame.c_str());
  return true;
}

vector5d EKF::odom_observation_maker(const nav_msgs::msg::Odometry & observation)
{
  vector5d z;
  z.setZero();

  z(0) = observation.pose.pose.position.x;
  z(1) = observation.pose.pose.position.y;
  z(2) = std::atan2(
    2.0 * (observation.pose.pose.orientation.w * observation.pose.pose.orientation.z +
           observation.pose.pose.orientation.x * observation.pose.pose.orientation.y),
    1.0 - 2.0 * (observation.pose.pose.orientation.y * observation.pose.pose.orientation.y +
                 observation.pose.pose.orientation.z * observation.pose.pose.orientation.z));

  z(3) = observation.twist.twist.linear.x;
  z(4) = observation.twist.twist.angular.z;

  return z.eval();
}

double EKF::calc_dt(rclcpp::Time & current, rclcpp::Time & prev)
{
  // was (current.seconds() + current.nanoseconds() * 1e-9) - (prev...): seconds() and nanoseconds() each
  // hold the whole time, so that counted it twice and every dt came out doubled
  return (current - prev).seconds();
}

void EKF::init_time()
{
  prev_update_time = this->now();
  time_init = true;
}

void EKF::publish_state()
{
  EKF::publish_odom();

  if (should_pub_tf) {
    EKF::publish_tf();
  }
}

void EKF::publish_odom()
{
  nav_msgs::msg::Odometry ekf_msg;

  // lowpass filtering
  double lowpass_speed = EKF::velocity_lowPass(mu(state_index::v));
  mu(state_index::v) = lowpass_speed;

  ekf_msg.child_frame_id = child_frame;
  ekf_msg.header.frame_id = header_frame;
  ekf_msg.header.stamp = prev_update_time;

  tf2::Quaternion q;
  q.setRPY(0, 0, mu(state_index::theta));

  ekf_msg.pose.pose.position.x = mu(state_index::x);
  ekf_msg.pose.pose.position.y = mu(state_index::y);

  ekf_msg.pose.pose.orientation.x = q.x();
  ekf_msg.pose.pose.orientation.y = q.y();
  ekf_msg.pose.pose.orientation.z = q.z();
  ekf_msg.pose.pose.orientation.w = q.w();

  ekf_msg.twist.twist.linear.x = mu(state_index::v);
  ekf_msg.twist.twist.angular.z = mu(state_index::theta_dot);

  if (debug_mode) {
    RCLCPP_INFO(this->get_logger(), "covariance matrix sigma t");
    for (int i = 0; i < 7; i++) {
      RCLCPP_INFO(
        this->get_logger(),
        " %f %f %f %f %f %f %f",
        sigma_t(i, 0),
        sigma_t(i, 1),
        sigma_t(i, 2),
        sigma_t(i, 3),
        sigma_t(i, 4),
        sigma_t(i, 5),
        sigma_t(i, 6));
    }
  }

  // bundle in the covariance with the mesurments

  // pose covariance
  ekf_msg.pose.covariance.at(0) = sigma_t(0, 0);  // x
  ekf_msg.pose.covariance.at(7) = sigma_t(1, 1);  // y
  ekf_msg.pose.covariance.at(35) = sigma_t(2, 2);  // yaw

  ekf_msg.twist.covariance.at(0) = sigma_t(3, 3);  // v
  ekf_msg.twist.covariance.at(35) = sigma_t(4, 4);  // thata_dot or angular velocity about z axis

  ekf_pub->publish(ekf_msg);
}

void EKF::publish_tf()
{
  geometry_msgs::msg::TransformStamped t;

  t.child_frame_id = child_frame;
  t.header.frame_id = header_frame;
  t.header.stamp = prev_update_time;

  t.transform.translation.x = mu(state_index::x);
  t.transform.translation.y = mu(state_index::y);
  t.transform.translation.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, mu(state_index::theta));

  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf_broadcaster->sendTransform(t);
}

double EKF::velocity_lowPass(double current_speed)
{
  double lowpass_speed = current_speed * alpha + (1 - alpha) * prev_speed;
  prev_speed = lowpass_speed;

  return lowpass_speed;
}

void EKF::init_params()
{
  // topic decliration
  this->declare_parameter<std::string>("ekf_topic", "/hal/ekf_odom");
  this->declare_parameter<std::string>("odom_topic", "/hal/odom");
  this->declare_parameter<std::string>("imu_topic", "/hal/imu");
  this->declare_parameter<std::string>("ackermann_topic", "/hal/ackermann_cmd");

  // frames declaration
  this->declare_parameter<std::string>("child_frame", "base_link");
  this->declare_parameter<std::string>("header_frame", "odom");
  this->declare_parameter<bool>("publish_tf", true);

  // physicl quantity declaration
  this->declare_parameter<double>("wheel_base", 0.3302);

  // low pass filter
  this->declare_parameter<double>("alpha", 0.55);

  // output rate (Hz) of the odometry and TF; <= 0 publishes after every update, as before
  this->declare_parameter<double>("publish_rate_hz", 100.0);

  // false: theta integrates theta_dot, which the IMU gyro corrects. true: the old bicycle model
  this->declare_parameter<bool>("yaw_from_steering", false);

  // debug mode
  this->declare_parameter<bool>("debug_mode", false);

  // mu inital declaration
  this->declare_parameter<double>("inital_x", 0.0);
  this->declare_parameter<double>("inital_y", 0.0);
  this->declare_parameter<double>("inital_theta", 0.0);
  this->declare_parameter<double>("inital_velocity", 0.0);
  this->declare_parameter<double>("inital_theta_dot", 0.0);
  this->declare_parameter<double>("inital_ax", 0.0);
  this->declare_parameter<double>("inital_ay", 0.0);

  // inital covariance matrix declaration
  this->declare_parameter<double>("x_cov", 10.0);
  this->declare_parameter<double>("y_cov", 10.0);
  this->declare_parameter<double>("theta_cov", 10.0);
  this->declare_parameter<double>("velocity_cov", 10.1);
  this->declare_parameter<double>("theta_dot_cov", 10.0);
  this->declare_parameter<double>("ax_cov", 10.0);
  this->declare_parameter<double>("ay_cov", 10.0);

  // declare the process noise matrix
  this->declare_parameter<double>("R_x", 1.0);
  this->declare_parameter<double>("R_y", 1.0);
  this->declare_parameter<double>("R_theta", 5.0);
  this->declare_parameter<double>("R_v", 1.0);
  this->declare_parameter<double>("R_theta_dot", 5.0);
  this->declare_parameter<double>("R_ax", 3.50);
  this->declare_parameter<double>("R_ay", 3.50);

  // declare the sensor noise matrix
  this->declare_parameter<double>("Q_odom_x", 1.5);
  this->declare_parameter<double>("Q_odom_y", 1.5);
  this->declare_parameter<double>("Q_odom_theta", 3.0);
  this->declare_parameter<double>("Q_odom_v", 0.75);
  this->declare_parameter<double>("Q_odom_theta_dot", 3.0);

  // Q_imu_theta is no longer used: the RealSense IMU has no orientation
  this->declare_parameter<double>("Q_imu_theta_dot", 0.5);
  this->declare_parameter<double>("Q_imu_ax", 0.65);
  this->declare_parameter<double>("Q_imu_ay", 0.65);

  // topic retrival
  odom_topic = this->get_parameter("odom_topic").as_string();
  ekf_topic = this->get_parameter("ekf_topic").as_string();
  imu_topic = this->get_parameter("imu_topic").as_string();
  ackermann_topic = this->get_parameter("ackermann_topic").as_string();
  should_pub_tf = this->get_parameter("publish_tf").as_bool();

  // frame retrival
  child_frame = this->get_parameter("child_frame").as_string();
  header_frame = this->get_parameter("header_frame").as_string();

  // pysical quantity retrival
  wheel_base = this->get_parameter("wheel_base").as_double();

  // low pass filter
  alpha = this->get_parameter("alpha").as_double();

  publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
  yaw_from_steering = this->get_parameter("yaw_from_steering").as_bool();

  // debug mode
  debug_mode = this->get_parameter("debug_mode").as_bool();

  // initalize mu (starting state vector)
  mu << this->get_parameter("inital_x").as_double(), this->get_parameter("inital_y").as_double(),
    this->get_parameter("inital_theta").as_double(), this->get_parameter("inital_velocity").as_double(),
    this->get_parameter("inital_theta_dot").as_double(), this->get_parameter("inital_ax").as_double(),
    this->get_parameter("inital_ay").as_double();

  // initalize the covariance matrix
  sigma_t.setZero();
  sigma_t(0, 0) = this->get_parameter("x_cov").as_double();
  sigma_t(1, 1) = this->get_parameter("y_cov").as_double();
  sigma_t(2, 2) = this->get_parameter("theta_cov").as_double();
  sigma_t(3, 3) = this->get_parameter("velocity_cov").as_double();
  sigma_t(4, 4) = this->get_parameter("theta_dot_cov").as_double();
  sigma_t(5, 5) = this->get_parameter("ax_cov").as_double();
  sigma_t(6, 6) = this->get_parameter("ay_cov").as_double();

  // initalize the sensor noice
  Q_imu.setZero();
  Q_odom.setZero();

  Q_odom(0, 0) = this->get_parameter("Q_odom_x").as_double();
  Q_odom(1, 1) = this->get_parameter("Q_odom_y").as_double();
  Q_odom(2, 2) = this->get_parameter("Q_odom_theta").as_double();
  Q_odom(3, 3) = this->get_parameter("Q_odom_v").as_double();
  Q_odom(4, 4) = this->get_parameter("Q_odom_theta_dot").as_double();

  Q_imu(0, 0) = this->get_parameter("Q_imu_theta_dot").as_double();
  Q_imu(1, 1) = this->get_parameter("Q_imu_ax").as_double();
  Q_imu(2, 2) = this->get_parameter("Q_imu_ay").as_double();

  // initalize the process noise
  R.setZero();
  R(0, 0) = this->get_parameter("R_x").as_double();
  R(1, 1) = this->get_parameter("R_y").as_double();
  R(2, 2) = this->get_parameter("R_theta").as_double();
  R(3, 3) = this->get_parameter("R_v").as_double();
  R(4, 4) = this->get_parameter("R_theta_dot").as_double();
  R(5, 5) = this->get_parameter("R_ax").as_double();
  R(6, 6) = this->get_parameter("R_ay").as_double();

  // initalize the H jacobian matrix for the odom sensor model (the IMU one is built per update)
  H_odom.setZero();

  for (int i = 0; i < 5; i++) {
    H_odom(i, i) = 1.0;
  }

  I7.setIdentity();

  RCLCPP_INFO(this->get_logger(), "initalized the params");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKF>());
  rclcpp::shutdown();
  return 0;
}
