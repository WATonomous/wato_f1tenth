#include "ekf_node.hpp"

EKF_NODE::EKF_NODE () : Node ("ekf_node") {

  //intalize the pubs and sub

  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/wheel_odom",10,std::bind(&EKF_NODE::odomCallBack,this,std::placeholders::_1));

  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    "/autodrive/f1tenth_1/imu",10,std::bind(&EKF_NODE::imuCallBack,this,std::placeholders::_1));

  steering_sub = this->create_subscription<std_msgs::msg::Float32>(
    "/autodrive/f1tenth_1/steering", 10, std::bind(&EKF_NODE::steeringCallBack,this,std::placeholders::_1));

  Throtel_sub = this->create_subscription<std_msgs::msg::Float32>(
    "/autodrive/f1tenth_1/throttle",10,std::bind(&EKF_NODE::throtelCallBack,this,std::placeholders::_1));

  ekf_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/ekf/odom",10);


  //initalize 
  EKF_NODE::initalize();

}

/**
 * @brief
 * stores the most recent imu message for EKF
 */
void EKF_NODE::imuCallBack (sensor_msgs::msg::Imu::SharedPtr msg) {
  current_imu = *msg;
}

/**
 * @brief
 * stores the most recent steering message for EKF
 */
void EKF_NODE::steeringCallBack(std_msgs::msg::Float32::SharedPtr msg) {
  current_steering = *msg;
}

void EKF_NODE::throtelCallBack(std_msgs::msg::Float32::SharedPtr msg) {
  current_throtel = *msg;
}

/**
 * @brief
 * The main callback function that triggers EKF state estimation.
 * 
 * Uses the first odometry message to initialize time t = t - 1,
 * along with the state vector μ at t = t - 1.
 * 
 * Calculates Δt and performs NaN checks to ensure the filter does not collapse.
 * 
 * @param msg The odometry callback message.
 */
void EKF_NODE::odomCallBack(nav_msgs::msg::Odometry::SharedPtr msg) {

  //initalize the first instance of time
  if (!intalized_time) {

    t_previous = msg->header.stamp;
    intalized_time = true;

    //initalize mu
    mu->Zero();
    (*mu)(State_space::X) = msg->pose.pose.position.x;
    (*mu)(State_space::Y) = msg->pose.pose.position.y;

    double theta = std::atan2(2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
    1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z));

    (*mu)(State_space::THETA) = theta;

    //skip this instance of the filter
    return;
  }

  rclcpp::Time t_current = msg->header.stamp;

  //calculate the time difference
  DT = std::abs((t_previous.seconds() + t_previous.nanoseconds() * 1e-9) - (t_current.seconds() + t_current.nanoseconds() * 1e-9));

  //update state with ekf
  vec7d new_mu;
  matrix7d new_sigma_t;
  new_mu = new_mu.Zero();
  new_sigma_t = new_sigma_t.Zero();

  EKF_NODE::ekf_pass(*msg, *mu, *sigma_t,new_mu,new_sigma_t);

  //error checking to ensure nan don't propogate
  if (!new_mu.allFinite() || !new_sigma_t.allFinite()) {
    if (!new_mu.allFinite()) RCLCPP_INFO(this->get_logger(),"mu is corrupted");
    if (!new_sigma_t.allFinite()) RCLCPP_INFO (this->get_logger(),"sigma is corupted");
    return;
  }

  *mu = new_mu;
  *sigma_t = new_sigma_t;
  t_previous = t_current;

  //normalize the yaw
  (*mu)(State_space::THETA) = std::atan2(std::sin((*mu)(2)),std::cos((*mu)(2)));

  //publish the new state
  EKF_NODE::publishOdom(*msg);

}

/**
 * @brief
 * Publishes the current EKF-estimated state as a `nav_msgs::msg::Odometry` message.
 * 
 * This function takes in a reference odometry message (for timestamp, frame info),
 * and publishes a new odometry message with the predicted EKF state.
 * The position and orientation are populated from the current state vector `mu`.
 * The velocity components are also included in the twist field.
 * 
 * @param odom The reference odometry message used for frame ID and timestamp.
 */
void EKF_NODE::publishOdom(const nav_msgs::msg::Odometry &odom) {

  nav_msgs::msg::Odometry ekf_msg;

  ekf_msg.child_frame_id = odom.child_frame_id;
  ekf_msg.header.frame_id = odom.header.frame_id;
  ekf_msg.header.stamp = odom.header.stamp;

  ekf_msg.pose.pose.position.x = (*mu)(State_space::X); 
  ekf_msg.pose.pose.position.y = (*mu)(State_space::Y);
  ekf_msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0,0,(*mu)(2));
  ekf_msg.pose.pose.orientation.x = q.x();  
  ekf_msg.pose.pose.orientation.y = q.y();
  ekf_msg.pose.pose.orientation.z = q.z();
  ekf_msg.pose.pose.orientation.w = q.w();
  
  ekf_msg.twist.twist.linear.x = (*mu)(State_space::V);
  ekf_msg.twist.twist.angular.z = (*mu)(State_space::THETA_DOT);


  //RCLCPP_INFO(this->get_logger(),"predicted velocity : %f ",mu(State::V));

  ekf_odom_pub->publish(ekf_msg);

}

/**
 * @brief
 * Performs the Extended Kalman Filter (EKF) update using:
 * the current control input, current observation, previous state mean, 
 * and previous covariance.
 * 
 * @param odom The current wheel odometry message at time t.
 * @param mu_p The predicted state (mean) at time t - 1.
 * @param sigma_t_p The predicted covariance at time t - 1.
 * @param new_mu The updated state (mean) at time t. (output)
 * @param new_sigma_t The updated covariance at time t. (output)
 * 
 * @return 
 * The updated mean and covariance at time t.
 */
void EKF_NODE::ekf_pass(const nav_msgs::msg::Odometry &odom, const vec7d &mu_p,const matrix7d &sigma_t_p,vec7d &new_mu, matrix7d &new_simga_t) {

  //get the jacobian & observation vector
  vec7d Z = EKF_NODE::observationCreator(odom,current_imu);
  matrix7d G = EKF_NODE::calculateJacobianG(mu_p,current_steering);

  //predict step
  vec7d mu_bar = EKF_NODE::modelUpdate(mu_p,current_steering);
  matrix7d sigma_t_bar = G * sigma_t_p * G.transpose() + R;

  //correction step
  matrix7d S = H * sigma_t_bar * H.transpose() + Q;
  S = S.inverse();
  matrix7d K = sigma_t_bar * H.transpose() * S;

  new_mu = mu_bar + K * (Z - EKF_NODE::observationMapper(mu_bar));
  new_simga_t = (I7 - K*H) * sigma_t_bar;

}

/**
 * @brief
 * Takes in the current wheel odometry and IMU data,
 * and produces the corresponding observation vector.
 * 
 * @param wheel_odom The current wheel odometry data.
 * @param imu The current IMU data.
 * 
 * @return
 * The observation vector Z 
 * (x, y, θ, θ_imu, ω_imu, a_x_imu, a_y_imu).
 */
vec7d EKF_NODE::observationCreator(const nav_msgs::msg::Odometry wheel_odom,const sensor_msgs::msg::Imu imu) {

  //calculate yaw from odom
  double theta = std::atan2(2.0 * (wheel_odom.pose.pose.orientation.w * wheel_odom.pose.pose.orientation.z + wheel_odom.pose.pose.orientation.x * wheel_odom.pose.pose.orientation.y),
    1.0 - 2.0 * (wheel_odom.pose.pose.orientation.y * wheel_odom.pose.pose.orientation.y + wheel_odom.pose.pose.orientation.z * wheel_odom.pose.pose.orientation.z));
  
  //calculate yaw from imu
  double theta_imu = std::atan2(2.0 * (imu.orientation.w * imu.orientation.z + imu.orientation.x * imu.orientation.y),
    1.0 - 2.0 * (imu.orientation.y * imu.orientation.y + imu.orientation.z * imu.orientation.z));

  //create and populate the observation vector
  vec7d observation; 

  observation << 
    wheel_odom.pose.pose.position.x,    
    wheel_odom.pose.pose.position.y,
    theta,
    theta_imu,
    imu.angular_velocity.z,
    imu.linear_acceleration.x,
    imu.linear_acceleration.y 
  ;
    
  return observation;
}

/**
 * @brief 
 * Maps the state space to the observation space 
 * as required for the state correction step.
 * 
 * @param predicted_state The state predicted by the motion model (g).
 * 
 * @return
 * The state vector mapped to the observation space.
 */
vec7d EKF_NODE::observationMapper(const vec7d &predicted_state) {

  vec7d observation_z;

  observation_z << 
    predicted_state(0),
    predicted_state(1),
    predicted_state(2),
    predicted_state(2),
    predicted_state(4),
    predicted_state(5),
    predicted_state(6);

  return observation_z;
}

/**
 * @brief 
 * Calculates the 7x7 Jacobian matrix for the motion model update function (g)
 * at time t - 1. This is part of the linearization process in the EKF.
 * The Jacobian consists of the partial derivatives of the motion model g.
 * 
 * @param previous_state The state of the robot at time t - 1.
 * @param steering_angle The current steering angle of the car.
 * 
 * @return
 * The Jacobian matrix G used in the EKF.
 */
matrix7d EKF_NODE::calculateJacobianG(const vec7d &previous_state,const std_msgs::msg::Float32 &steering_angle) {

  double theta = previous_state (2), 
    v = current_throtel.data * MAX_VELOCITY, ax = previous_state(5), 
    ay = previous_state(6), phi = steering_angle.data; 

  matrix7d jacobian;
  jacobian.Zero();

  //1st row
  jacobian(0,0) = 1.0;
  jacobian(0,2) = -v * std::sin(theta) * DT;
  jacobian(0,3) = std::cos(theta) * DT;
  jacobian(0,5) = 0.5 * std::pow(DT,2);

  //2nd row
  jacobian(1,1) = 1.0;
  jacobian(1,2) = v * std::cos(theta) * DT;
  jacobian(1,3) = std::sin(theta) * DT;
  jacobian(1,6) = 0.5 * std::pow(DT,2);

  //3rd row
  jacobian(2,2) = 1.0;
  jacobian(2,3) = (std::tan(phi) * DT) / L_WB;

  //4th row
  jacobian(3,2) = ay * std::cos(theta) * DT - ax * std::sin(theta) * DT;
  jacobian(3,3) = 1.0;
  jacobian(3,5) = std::cos(theta) * DT;
  jacobian(3,6) = std::sin(theta) * DT;

  //5th row
  jacobian(4,3) = std::tan(phi) / L_WB;

  //6th row
  jacobian(5,5) = 1.0;

  //7th row
  jacobian(6,6) = 1.0; 

  return jacobian;

}

/**
 * @brief 
 * Propagates the state forward to time t using a tricycle model.
 * 
 * @param current_state The state of the robot at time t-1.
 * @param steering_angle The steering input at time t.
 * 
 * @return 
 * The predicted state at time t
 * (x,y,theta,v,theta_dot,ax,ay).
 */

vec7d EKF_NODE::modelUpdate (const vec7d &current_state,const std_msgs::msg::Float32 &steering_angle) {

  double x = current_state (State_space::X), y = current_state (State_space::Y), theta = current_state (State_space::THETA), 
    v = current_throtel.data * MAX_VELOCITY, ax = current_state(State_space::AX), 
    ay = current_state(State_space::AY), phi = steering_angle.data; 

  vec7d predicted_state;

  predicted_state << 
    x + v * std::cos(theta) * DT + 0.5 * ax * std::pow(DT,2),
    y + v * std::sin(theta) * DT + 0.5 * ay * std::pow(DT,2),
    theta + (v/L_WB) * std::tan(phi) * DT,
    v + ax * std::cos(theta) * DT + ay * std::sin(theta) * DT,
    (v/L_WB) * std::tan(phi),
    ax,
    ay
  ;

  return predicted_state;

}

/**
 * @brief
 * Initializes the following variables: 
 * sigma_t (initial covariance), mu (initial state),
 * R (process noise covariance), Q (observation noise covariance), 
 * H (observation Jacobian), and I7 (a 7x7 identity matrix).
 * 
 * These covariances and the initial state should be 
 * adjusted based on the scenario — either empirically 
 * or based on intuition.
 */
void EKF_NODE::initalize() {

  //initalize starting covariance and nose
  sigma_t = new matrix7d ();
  mu = new vec7d ();

  sigma_t->Zero();
  R.Zero();
  Q.Zero();
  H.Zero();
  I7 = I7.Identity();

  for (int i = 0; i < 7; i++) 
    (*sigma_t)(i,i) = R(i,i) = 0.1;

  //process noise
  R(0,0) = 0.01;
  R(1,1) = 0.01;
  R(2,2) = 0.05;

  //set the sensor noise
  Q(0,0) = 17.0;
  Q(1,1) = 17.0;
  Q(2,2) = 170.0;
  Q(3,3) = 6.85e-07;
  Q(4,4) = 1.09e-07;
  Q(5,5) = 0.0015;
  Q(6,6) = 0.0015;

  //compute the jacobian of the observation model
  H(0,0) = 1.0;
  H(1,1) = 1.0;
  H(2,2) = 1.0;
  H(3,2) = 1.0;
  H(4,4) = 1.0;
  H(5,5) = 1.0;
  H(6,6) = 1.0;  

  RCLCPP_INFO(this->get_logger(),"intalized startinc conditions ");

}


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKF_NODE>());
  rclcpp::shutdown();
  return 0;
}

