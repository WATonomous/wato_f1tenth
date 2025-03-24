#include <chrono>
#include <memory>

#include <cmath>

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "rotations.h"

using namespace Eigen;
using namespace std;


#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "estimation_node.hpp"

EstimationNode::EstimationNode() : Node("estimation"), estimation_(robot::EstimationCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&EstimationNode::publishMessage, this));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  
//   lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//       "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&EstimationNode::laserCallback, this, std::placeholders::_1));


// Create a subscriber to the IMU topic
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data",  // Change this to the name of your IMU topic
         10,  // Queue size
         std::bind(&IEstimationNode::imuCallback, this, std::placeholders::_1)

  estimation_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/estimation", 10);
    );

  // Initialize constants
        C_li << 0.99376, -0.09722, 0.05466,
                0.09971, 0.99401, -0.04475,
               -0.04998, 0.04992, 0.9975;

        t_i_li = Vector3d(0.5, 0.1, 0.5);
        var_imu_f = 0.10;
        var_imu_w = 0.10;
        var_gnss = 0.10;
        var_lidar = 2.00;
        g = Vector3d(0, 0, -9.81);

        l_jac = MatrixXd::Zero(9, 6);
        l_jac.block<6, 6>(3, 0) = MatrixXd::Identity(6, 6);

        h_jac = MatrixXd::Zero(3, 9);
        h_jac.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);
        
        //allocated space for the state variables : 
        // Define IMU state variables as global variables
        Eigen::Vector3d p_est = Eigen::Vector3d::Zero();   // Position [x, y, z]
        Eigen::Vector3d v_est = Eigen::Vector3d::Zero();   // Velocity [vx, vy, vz]
        Eigen::Quaterniond q_est = Eigen::Quaterniond::Identity(); // Orientation (quaternion)
        Eigen::Matrix<double, 9, 9> p_cov = Eigen::Matrix<double, 9, 9>::Zero(); // 9x9 Position and velocity covariance matrix
        Eigen::Vector3d imu_w = Eigen::Vector3d::Zero();   // Angular velocity [wx, wy, wz]
        Eigen::Vector3d imu_f = Eigen::Vector3d::Zero();   // Linear acceleration [fx, fy, fz]

        Eigen::Vector3d imu_w_past = Eigen::Vector3d::Zero();   // Angular velocity [wx, wy, wz]
        Eigen::Vector3d imu_f_past = Eigen::Vector3d::Zero();   // Linear acceleration [fx, fy, fz]

}

//need angular velocity to replace imu_w
// Callback function to handle incoming IMU messages
//assumptions: imu at centre of mass, and perfectly level.
void EstimationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    //diff in time between messages
    rclcpp::Time current_time = this->now(); // Get current ROS time

    // i dont think this is necessary
    // if (last_time_.nanoseconds() == 0) {
    //     last_time_ = current_time; // Set first received time
    // }

    double delta_t = (current_time - last_time_).nanoseconds() / 1.0e6; // Δt in milliseconds
    last_time_ = current_time; // Update last received time

    // Store individual components of angular velocity (imu_w_)
    imu_w << msg->angular_velocity.x, 
               msg->angular_velocity.y, 
               msg->angular_velocity.z;

    // Store individual components of linear acceleration (imu_f_)
    imu_f << msg->linear_acceleration.x, 
              msg->linear_acceleration.y, 
              msg->linear_acceleration.z;

    // Compute magnitudes for angular velocity (w) and linear acceleration (f)
    double w_magnitude = imu_w.norm(); // Euclidean norm of angular velocity
    double f_magnitude = imu_f.norm(); // Euclidean norm of linear acceleration

    // Print values (optional)
    RCLCPP_INFO(this->get_logger(), 
       "IMU w: [%.3f, %.3f, %.3f] (magnitude: %.3f), IMU f: [%.3f, %.3f, %.3f] (magnitude: %.3f)", 
       imu_w.x(), imu_w.y(), imu_w.z(), w_magnitude,
       imu_f.x(), imu_f.y(), imu_f.z(), f_magnitude);

    // // converts ROS quaternion to Eigen Quaternion
    // Eigen::Quaterniond imu_orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    // q_est.row(0) = imu_orientation.coeffs();  // store in q_est

    // // extract linear acceleration
    // Eigen::Vector3d imu_accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    // // inegrate acceleration to update velocity (100hz updates, idk how fast the clock is might be cnaged later)
    // v_est.row(0) += imu_accel.transpose() * 0.01;

    // // log transformed data
    // RCLCPP_INFO(this->get_logger(), "Updated IMU Data -> Velocity: [%.2f, %.2f, %.2f]",
    //             v_est(0,0), v_est(0,1), v_est(0,2));

    //             // extract linear acceleration, remove g from z, mulitply by mass.
    // imu_f.row(0) = m * imu_accel.coeefs();
    // Eigen::Vector3d imu_angular(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    // imu_w.row(0) = imu_angular.coeffs();

    // //(for imuw, imuf, vest, qest, pest and such)

    mainLoop( delta_t, p_est, v_est, q_est, imu_w_past, imu_f_past, p_cov); 

    imu_w_past = imu_w;
    imu_f_past = imu_f;


    //make class variables for these inputs, except delta t
    // pretty sure only imu_w and imu_f need before and after

}


void EstimationNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
 
    if (msg->ranges.empty()) return;

    // //diff in time between messages
    // rclcpp::Time current_time = this->now(); // Get current ROS time


    // double delta_t = (current_time - last_time_).nanoseconds() / 1.0e6; // Δt in milliseconds
    // last_time_ = current_time; // Update last received time

    // get 1st LIDAR measurement (assume closest)?
    double angle = msg->angle_min;
    double range = msg->ranges[0];

    // cinvert to careteseian coordinates (LIDAR frame)
    Eigen::Vector3d lidar_point(range * cos(angle), range * sin(angle), 0.0);

    // transform LIDAR to IMU frame
    Eigen::Vector3d imu_position = (C_li * lidar_point) + t_i_li;
    
    // store updated position
    p_est.row(0) = imu_position.transpose();
    
    //sensor var ??
    measurementUpdate(sensor_var, p_cov_check, y_k, p_check, v_check, q_check);

    // log the transform
    RCLCPP_INFO(this->get_logger(), "Transformed LIDAR Position -> [%.2f, %.2f, %.2f]",
                imu_position.x(), imu_position.y(), imu_position.z());


}

// Noemie: must be changed to handle actual data structure from inputs
// getting accurate corrected positions from lidar requires an algorithm that resembles slam, point cloud etc..
//which makes this redundant and not worth implementing
tuple<Vector3d, Vector3d, Quaternion, MatrixXd> EstimationNode::measurementUpdate(
        double sensor_var, const MatrixXd &p_cov_check, const Vector3d &y_k,
        const Vector3d &p_check, const Vector3d &v_check, const Quaternion &q_check) {

        Matrix3d r_cov = Matrix3d::Identity() * sensor_var;
        MatrixXd k_gain = p_cov_check * h_jac.transpose() *
                          (h_jac * p_cov_check * h_jac.transpose() + r_cov).inverse();

        VectorXd error_state = k_gain * (y_k - p_check);

        Vector3d p_hat = p_check + error_state.head(3);
        Vector3d v_hat = v_check + error_state.segment(3, 3);
        Quaternion q_hat = Quaternion(error_state.tail(3)).quat_mult_left(q_check);

        MatrixXd p_cov_hat = (MatrixXd::Identity(9, 9) - k_gain * h_jac) * p_cov_check;


        publishMessage(p_hat, v_hat, q_hat);

        //update est values here? 
        //row not necesary
        // p_est.row(k) = p_hat;
        // v_est.row(k) = v_hat;
        // q_est.row(k) = q_hat;
        // p_cov[k] = p_cov_hat;

        // i think we should return this in the publisher
        //shouldnt/why arent these values going into the "est" values after a measurement update
        return make_tuple(p_hat, v_hat, q_hat, p_cov_hat);
}


//I THINK RESTRUCTURE NEEDED
//Note: delta t now comes from call back functions NB: doesnt need to come from lasercallback (since only used for measurement update)
//Now we need to define then and now variables in our class,
//this main loop function will be editted to have parameters,
//i think the parameters will be delta t, and the then and now variables (for imuw, imuf, vest, qest, pest and such)

//pest, vest, qest, pcov are only updated after measurement update + lidar if u look at github
//lidar is only used for measurment update
// so i think measurement update should be called from lasercallback
//for this we would need the "check" and "est" values to have class variables 

//while main loop is called from imucallback
//and imuw and imuf have past and current class variables.

//pretty sure we dont need vest from imu and pest from lasercall back
//measurement update results in updated "est" values, which uses i think distance from lidar and "check" values

//where "check" values are predicted, "hat" values are corrected, "est" values are current state
void EstimationNode::MotionModel() {
        
        //we need two vector variables for imuw imuf vest qest pest
        //they will refect past and current, after each loop past becomes current, then we get new data
        // for everytime we get sensor data
        //so this for loop is replaced by placing it in imucallback
        //for (int k = 1; k < imu_f.rows(); ++k) {
          
          //PREDICTION
          //Update state with IMU inputs
          //SANIA: 
          // q.. variable only need one variable since the past variable is used to predict the next/current one
          //imuf & imu_w need past and current values.
            Quaternion q_prev(q_est);
            Quaternion q_curr(imu_w_past * delta_t);
            Matrix3d c_ns = q_prev.to_mat();
            Vector3d f_ns = (c_ns * imu_f_past.transpose()) + g;

            Vector3d p_check = p_est.transpose() + delta_t * v_est.transpose() +
                               0.5 * delta_t * delta_t * f_ns;
            Vector3d v_check = v_est.transpose() + delta_t * f_ns;
            Quaternion q_check = q_prev.quat_mult_left(q_curr);

            // Linearize motion model and compute Jacobians
            MatrixXd f_jac = MatrixXd::Identity(9, 9);
            f_jac.block<3, 3>(0, 3) = MatrixXd::Identity(3, 3) * delta_t;
            f_jac.block<3, 3>(3, 6) = -skew_symmetric(c_ns * imu_f_past.transpose()) * delta_t;
          // propogate uncertainty
            MatrixXd q_cov = MatrixXd::Zero(6, 6);
            q_cov.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3) * delta_t * delta_t * var_imu_f;
            q_cov.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3) * delta_t * delta_t * var_imu_w;

            MatrixXd p_cov_check = f_jac * p_cov * f_jac.transpose() + l_jac * q_cov * l_jac.transpose();

            //CORRECTION
            // think this is unecessary if we call measurement update in lasercallback
            // GNSS and LIDAR updates
            // if (find(gnss_t.begin(), gnss_t.end(), imu_f(k, 0)) != gnss_t.end()) {
            //     int gnss_i = distance(gnss_t.begin(), find(gnss_t.begin(), gnss_t.end(), imu_f(k, 0)));
            //     tie(p_check, v_check, q_check, p_cov_check) =
            //         measurementUpdate(var_gnss, p_cov_check, gnss.row(gnss_i).transpose(), p_check, v_check, q_check);
            // }

            // if (find(lidar_t.begin(), lidar_t.end(), imu_f(k, 0)) != lidar_t.end()) {
            //     int lidar_i = distance(lidar_t.begin(), find(lidar_t.begin(), lidar_t.end(), imu_f(k, 0)));
            //     tie(p_check, v_check, q_check, p_cov_check) =
            //         measurementUpdate(var_lidar, p_cov_check, lidar.row(lidar_i).transpose(), p_check, v_check, q_check);
            // }

            // Update states
            //why is it check instead of hat values
            //row isn't necessary
            p_est = p_check.transpose();
            v_est = v_check.transpose();
            q_est = q_check.to_numpy();
            p_cov = p_cov_check;
        //}
}



//not sure this'll work its just a first try
void EstimationNode::publishMessage(const Eigen::Vector3d &p_est, const Eigen::Vector3d &v_est, const Eigen::Quaterniond &q_est) {
  auto message = nav_msgs::msg::Odometry();

  // Set timestamp and frame ID
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "odom";  // Change "map" to your relevant frame

  // Populate position
  message.pose.pose.position.x = p_est.x();
  message.pose.pose.position.y = p_est.y();
  message.pose.pose.position.z = p_est.z();

  // Populate orientation (ensure correct Eigen access)
  message.pose.pose.orientation.x = q_est.x();
  message.pose.pose.orientation.y = q_est.y();
  message.pose.pose.orientation.z = q_est.z();
  message.pose.pose.orientation.w = q_est.w();

  // Populate velocity
  message.twist.twist.linear.x = v_est.x();
  message.twist.twist.linear.y = v_est.y();
  message.twist.twist.linear.z = v_est.z();

  // Optionally, set covariance matrices (identity matrix as placeholder)
  message.pose.covariance.fill(0.0);
  message.twist.covariance.fill(0.0);

  RCLCPP_INFO(this->get_logger(), "Publishing estimation data (Pose & Velocity).");
  estimation_pub_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstimationNode>());
  rclcpp::shutdown();
  return 0;
}