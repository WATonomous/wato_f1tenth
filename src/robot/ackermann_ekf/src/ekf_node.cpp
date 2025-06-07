#include "ekf_node.hpp"

EKF_NODE::EKF_NODE () : Node ("ekf_node") {

  //intalize the pubs and sub

  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/wheel_odom",10,std::bind(&EKF_NODE::odomCallBack,this,std::placeholders::_1));

  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    "/autodrive/f1tenth_1/imu",10,std::bind(&EKF_NODE::imuCallBack,this,std::placeholders::_1));

  steering_sub = this->create_subscription<std_msgs::msg::Float32>(
    "/autodrive/f1tenth_1/steering", 10, std::bind(&EKF_NODE::steeringCallBack,this,std::placeholders::_1));

  ekf_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/ekf/odom",10);

  //initalize 
  EKF_NODE::initalize();

}

void EKF_NODE::imuCallBack (sensor_msgs::msg::Imu::SharedPtr msg) {
  current_imu = *msg;
}

void EKF_NODE::steeringCallBack(std_msgs::msg::Float32::SharedPtr msg) {
  current_steering = *msg;
}

void EKF_NODE::odomCallBack(nav_msgs::msg::Odometry::SharedPtr msg) {

  //initalize the first instance of time
  if (!intalized_time) {
    t_previous = msg->header.stamp;
    intalized_time = true;

    //initalize mu

    mu.Zero();
    mu(State::X) = msg->pose.pose.position.x;
    mu(State::Y) = msg->pose.pose.position.y;

    double theta = std::atan2(2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
    1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z));

    mu(State::THETA) = theta;

    //skip this instance of the filter
    return;
  }

  rclcpp::Time t_current = msg->header.stamp;

  //calculate the time difference
  DT = std::abs((t_previous.seconds() + t_previous.nanoseconds() * 1e-9) - (t_current.seconds() + t_current.nanoseconds() * 1e-9));
  t_previous = t_current;

  //update state with ekf
  EKF_NODE::ekf(*msg);

  //publish the new state
  EKF_NODE::publishOdom(*msg);

}

void EKF_NODE::publishOdom(const nav_msgs::msg::Odometry &odom) {

  nav_msgs::msg::Odometry ekf_msg;

  ekf_msg.child_frame_id = odom.child_frame_id;
  ekf_msg.header.frame_id = odom.header.frame_id;
  ekf_msg.header.stamp = odom.header.stamp;

  ekf_msg.pose.pose.position.x = mu(State::X); 
  ekf_msg.pose.pose.position.y = mu(State::Y);
  ekf_msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0,0,mu(State::THETA));
  ekf_msg.pose.pose.orientation.x = q.x();  
  ekf_msg.pose.pose.orientation.y = q.y();
  ekf_msg.pose.pose.orientation.z = q.z();
  ekf_msg.pose.pose.orientation.w = q.w();
  
  ekf_msg.twist.twist.linear.x = mu(State::V);
  ekf_msg.twist.twist.angular.z = mu(State::THETA_DOT);

  //RCLCPP_INFO(this->get_logger(),"predicted velocity : %f ",mu(State::V));

  ekf_odom_pub->publish(ekf_msg);

}

//for the break down how how this works look at the read me in package folder
void EKF_NODE::ekf(const nav_msgs::msg::Odometry &odom) {

  //calculate the jacobian G and observation vector Z
  vec7d Observation_Z = EKF_NODE::observationCreator(odom,current_imu); 
  matrix7d jacobian_G = calculateJacobianG (mu,current_steering);

  if (!check_one) {
    RCLCPP_INFO(this->get_logger(),"jacobian G");
    RCLCPP_INFO(this->get_logger(), " %f %f %f %f %f %f %f" , jacobian_G(0,0), jacobian_G(0,1), jacobian_G(0,2), jacobian_G(0,3), jacobian_G(0,4),jacobian_G(0,5),jacobian_G(0,6));
    RCLCPP_INFO(this->get_logger(), " %f %f %f %f %f %f %f" , jacobian_G(1,0), jacobian_G(1,1), jacobian_G(1,2), jacobian_G(1,3), jacobian_G(1,4),jacobian_G(1,5),jacobian_G(1,6));
    RCLCPP_INFO(this->get_logger(), " %f %f %f %f %f %f %f" , jacobian_G(2,0), jacobian_G(2,1), jacobian_G(2,2), jacobian_G(2,3), jacobian_G(2,4),jacobian_G(2,5),jacobian_G(2,6));
    RCLCPP_INFO(this->get_logger(), " %f %f %f %f %f %f %f" , jacobian_G(3,0), jacobian_G(3,1), jacobian_G(3,2), jacobian_G(3,3), jacobian_G(3,4),jacobian_G(3,5),jacobian_G(3,6));
    RCLCPP_INFO(this->get_logger(), " %f %f %f %f %f %f %f" , jacobian_G(4,0), jacobian_G(4,1), jacobian_G(4,2), jacobian_G(4,3), jacobian_G(4,4),jacobian_G(4,5),jacobian_G(4,6));
    RCLCPP_INFO(this->get_logger(), " %f %f %f %f %f %f %f" , jacobian_G(5,0), jacobian_G(5,1), jacobian_G(5,2), jacobian_G(5,3), jacobian_G(5,4),jacobian_G(5,5),jacobian_G(5,6));
    RCLCPP_INFO(this->get_logger(), " %f %f %f %f %f %f %f" , jacobian_G(6,0), jacobian_G(6,1), jacobian_G(6,2), jacobian_G(6,3), jacobian_G(6,4),jacobian_G(6,5),jacobian_G(6,6));
    
    RCLCPP_INFO(this->get_logger(),"observation Z");
    RCLCPP_INFO(this->get_logger(),"%f", Observation_Z(0));
    RCLCPP_INFO(this->get_logger(),"%f", Observation_Z(1));
    RCLCPP_INFO(this->get_logger(),"%f", Observation_Z(2));
    RCLCPP_INFO(this->get_logger(),"%f", Observation_Z(3));
    RCLCPP_INFO(this->get_logger(),"%f", Observation_Z(4));
    RCLCPP_INFO(this->get_logger(),"%f", Observation_Z(5));
    RCLCPP_INFO(this->get_logger(),"%f", Observation_Z(6));

    check_one = true;
  }
  //!!!predict step!!!

  // calculate the predicted state
  vec7d mu_bar = modelUpdate(mu,current_steering);

  //calculate the sigma_t_bar matrix
  matrix7d sigma_t_bar = jacobian_G * sigma_t * jacobian_G.transpose() + R;

  //!!!correction step!!!

  //solve for kalman gain 
  matrix7d S = H * sigma_t_bar * H.transpose() + Q;
  matrix7d kalman_gain = sigma_t_bar * H.transpose();
  kalman_gain = S.llt().solve(kalman_gain); // bassicaly solving Sx = K, x = K * S^-1

  //correct the state
  mu = mu_bar + kalman_gain * (Observation_Z - observationMapper(mu_bar));

  //calculate the covariance
  sigma_t = (I7 - kalman_gain*H) * sigma_t_bar;

}

//(x,y,z,theta,theta_imu,w_imu,ax_imu,ay_imu)
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

//maps a vector from state space into observation space
vec7d EKF_NODE::observationMapper(const vec7d &predicted_state) {
  return H * predicted_state;
}

//calculate G acording to the matrix shown in the read me / docs
matrix7d EKF_NODE::calculateJacobianG(const vec7d &current_state,const std_msgs::msg::Float32 &steering_angle) {

  double theta = current_state (2), 
    v = current_state (3), ax = current_state(5), 
    ay = current_state(6), phi = steering_angle.data; 

  matrix7d jacobian;
  jacobian.Zero();

  //1st row
  jacobian(0,0) = 1.0;
  jacobian(0,2) = -v * std::sin(theta) * DT;
  jacobian(0,3) = std::cos(theta) * DT;
  jacobian(0,5) = 0.5 * std::pow(DT,2);

  //2nd row
  jacobian(1,1) = 1.0;
  jacobian(1,2) = -v * std::sin(theta) * DT;
  jacobian(1,3) = std::cos(theta) * DT;
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

vec7d EKF_NODE::modelUpdate (const vec7d &current_state,const std_msgs::msg::Float32 &steering_angle) {

  double x = current_state (0), y = current_state (1), theta = current_state (2), 
    v = current_state (3), ax = current_state(5), 
    ay = current_state(6), phi = steering_angle.data; 

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

//initalization works, all matracies have proper values
void EKF_NODE::initalize() {

  //initalize starting covariance and nose
  sigma_t.Zero();
  R.Zero();
  Q.Zero();
  H.Zero();
  I7 = I7.Identity();

  for (int i = 0; i < 7; i++) 
    sigma_t(i,i) = R(i,i) = 0.1;

  //set the sensor noise
  Q(0,0) = 0.1;
  Q(1,1) = 0.1;
  Q(2,2) = 0.1;
  Q(3,3) = 0.05;
  Q(4,4) = 0.05;
  Q(5,5) = 0.075;
  Q(6,6) = 0.075;

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

