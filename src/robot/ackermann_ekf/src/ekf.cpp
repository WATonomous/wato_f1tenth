#include "ekf.hpp"

EKF::EKF () : Node ("ekf_rt") {

    EKF::init_params();

    //subs
    control_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>
        (ackermann_topic, 10, std::bind(&EKF::control_callback, this, std::placeholders::_1));

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>
        (odom_topic, 10 , std::bind(&EKF::odom_callback, this, std::placeholders::_1));

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>
        (imu_topic, 10, std::bind(&EKF::imu_callback, this, std::placeholders::_1));

    //pubs
    ekf_pub = this->create_publisher<nav_msgs::msg::Odometry>(ekf_topic, 10);
    
}

void EKF::control_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr input) {

    if (!init_time) {
        EKF::init_time();
        return;
    }

    prediction current_predition = EKF::prediction_step(mu, sigma_t, *input);

    prev_input = *input; // save this regardless, a fail. always want the latest control input
    if (current_predition.did_error) {
        RCLCPP_INFO(this->get_logger(), "control_callback predition corrupt");
        return;
    }

    //save the previous state
    mu = current_predition.mu;
    sigma_t = current_predition.sigma_t;

    EKF::publish_odom();
    prev_update_time = current_predition.calc_time;
    
}

void EKF::imu_callback(const sensor_msgs::msg::Imu imu_msg) {

    if (!init_time) {
        EKF::init_time();
        return;
    }

    prediction current_predition = EKF::prediction_step(mu, sigma_t, prev_input);
    
    if (current_predition.did_error) {
        RCLCPP_INFO(this->get_logger(), "imu_prediction corrupt");
        return;
    }

    EKF::publish_odom();
    prev_update_time = current_predition.calc_time;
}

void EKF::odom_callback(const nav_msgs::msg::Odometry odom_msg) {

    if (!init_time) {
        EKF::init_time();
        return;
    }

    prediction current_predition = EKF::prediction_step(mu, sigma_t, prev_input);
    
    if (current_predition.did_error) {
        RCLCPP_INFO(this->get_logger(), "odom predition corrupt");
        return;
    }

    EKF::publish_odom();
    prev_update_time = current_predition.calc_time;
}

vector7d EKF::model_update(const vector7d &mu_prev, const ackermann_msgs::msg::AckermannDriveStamped &control_input, double dt_) {

}

matrix7d EKF::predict_sigma(const matrix7d &sigma_prev, const matrix7d &jacobian_g) {
    matrix7d sigma_predicted = jacobian_g * sigma_predicted * jacobian_g.transpose() + R;
    return sigma_predicted;
}

matrix7d EKF::calc_jacobian_g(const vector7d &mu_prev, double dt_) {

    matrix7d jacobian;
    jacobian.Zero();

    // 1st row
    jacobian(0, 0) = 1.0;
    jacobian(0, 2) = -prev_input.drive.speed * std::sin(mu_prev(state_index::theta)) * dt_;
    jacobian(0, 3) = std::cos(mu_prev(state_index::theta)) * dt_;
    jacobian(0, 5) = 0.5 * std::pow(dt_, 2);

    // 2nd row
    jacobian(1, 1) = 1.0;
    jacobian(1, 2) = prev_input.drive.speed * std::cos(mu_prev(state_index::theta)) * dt_;
    jacobian(1, 3) = std::sin(mu_prev(state_index::theta)) * dt_;
    jacobian(1, 6) = 0.5 * std::pow(dt_, 2);

    // 3rd row
    jacobian(2, 2) = 1.0;
    jacobian(2, 3) = (std::tan(prev_input.drive.steering_angle) * dt_) / wheel_base;

    // 4th row
    jacobian(3, 2) = mu_prev(state_index::ay) * std::cos(mu_prev(state_index::theta)) * dt_ - mu_prev(state_index::ax) * std::sin(mu_prev(state_index::theta)) * dt_;
    jacobian(3, 3) = 1.0;
    jacobian(3, 5) = std::cos(mu_prev(state_index::theta)) * dt_;
    jacobian(3, 6) = std::sin(mu_prev(state_index::theta)) * dt_;

    // 5th row
    jacobian(4, 3) = std::tan(prev_input.drive.steering_angle) / wheel_base;

    // 6th row
    jacobian(5, 5) = 1.0;

    // 7th row
    jacobian(6, 6) = 1.0;

    return jacobian;
}

prediction EKF::prediction_step(const vector7d &mu_prev, const matrix7d &sigma_prev, const ackermann_msgs::msg::AckermannDriveStamped &control_input){

    prediction current_prediction;
    
    rclcpp::Time current_time = this->now();
    double dt = EKF::calc_dt(current_time,prev_update_time);

    vector7d mu_bar = EKF::model_update(mu_prev,control_input,dt);
    matrix7d jacobian_G = EKF::calc_jacobian_g(mu_prev,dt);
    matrix7d sigma_bar = EKF::predict_sigma(sigma_prev,jacobian_G);

    if (!mu_bar.allFinite() || !sigma_bar.allFinite()) {
        current_prediction.did_error = true;
    } else {
        current_prediction.did_error = false;
        current_prediction.mu = mu_bar;
        current_prediction.sigma_t = sigma_bar;
        current_prediction.calc_time = current_time;
    }

    return current_prediction;

}

void EKF::init_time() {
    prev_update_time = this->now();
}

void EKF::init_params () {

    //topic decliration
    this->declare_parameter<std::string>("ekf_topic","/ekf/odom");
    this->declare_parameter<std::string>("odom_topic","/odom");
    this->declare_parameter<std::string>("imu_topic","/autodrive/f1tenth_1/imu");
    this->declare_parameter<std::string>("ackermann_topic","/ackermann_cmd");

    //frames declaration
    this->declare_parameter<std::string>("child_frame","base_link");
    this->declare_parameter<std::string>("header_frame","odom");

    //physicl quantity declaration
    this->declare_parameter<double>("wheel_base",0.3240);

    //mu inital declaration
    this->declare_parameter<double>("inital_x",0.7412);
    this->declare_parameter<double>("inital_y",3.1583);
    this->declare_parameter<double>("inital_theta",-M_PI/1);
    this->declare_parameter<double>("inital_velocity", 0.0);
    this->declare_parameter<double>("inital_theta_dot", 0.0);
    this->declare_parameter<double>("inital_ax", 0.0);
    this->declare_parameter<double>("intal_ay", 0.0);

    //inital covariance matrix declaration
    this->declare_parameter<double>("x_cov",0.1);
    this->declare_parameter<double>("y_cov",0.1);
    this->declare_parameter<double>("theta_cov",0.1);
    this->declare_parameter<double>("velocity_cov", 0.1);
    this->declare_parameter<double>("theta_dot_cov", 0.1);
    this->declare_parameter<double>("ax_cov", 0.1);
    this->declare_parameter<double>("ay_cov", 0.1);

    //declare the process noise matrix 
    this->declare_parameter<double>("R_x",0.1);
    this->declare_parameter<double>("R_y",0.1);
    this->declare_parameter<double>("R_theta",0.1);
    this->declare_parameter<double>("R_v",0.1);
    this->declare_parameter<double>("R_theta_dot",0.1);
    this->declare_parameter<double>("R_ax",0.1);
    this->declare_parameter<double>("R_ay",0.1);

    //declare the sensor noise matrix
    this->declare_parameter<double>("Q_odom_x",0.1);
    this->declare_parameter<double>("Q_odom_y",0.1);
    this->declare_parameter<double>("Q_odom_theta",0.1);
    this->declare_parameter<double>("Q_odom_v",0.1);
    this->declare_parameter<double>("Q_odom_theta_dot",0.1);

    this->declare_parameter<double>("Q_imu_theta",0.1);
    this->declare_parameter<double>("Q_imu_theta_dot",0.1);
    this->declare_parameter<double>("Q_imu_ax",0.1);
    this->declare_parameter<double>("Q_imu_ay",0.1);

    //topic retrival
    odom_topic = this->get_parameter("odom_topic").as_string();
    ekf_topic = this->get_parameter("ekf_topic").as_string();
    imu_topic = this->get_parameter("imu_topic").as_string();
    ackermann_topic = this->get_parameter("ackermann_topic").as_string();

    //frame retrival
    child_frame = this->get_parameter("child_frame").as_string();
    header_frame = this->get_parameter("header_frame").as_string();

    //pysical quantity retrival
    wheel_base = this->get_parameter("wheel_base").as_double();

    //initalize mu (starting state vector)
    mu << 
        this->get_parameter("inital_x").as_double(),
        this->get_parameter("inital_y").as_double(),
        this->get_parameter("inital_theta").as_double(),
        this->get_parameter("inital_velocity").as_double(),
        this->get_parameter("inital_theta_dot").as_double(),
        this->get_parameter("inital_ax").as_double(),
        this->get_parameter("inital_ay").as_double()
    ;

    //initalize the covariance matrix 
    sigma_t.Zero();
    sigma_t(0,0) = this->get_parameter("x_cov").as_double();
    sigma_t(1,1) = this->get_parameter("y_cov").as_double();
    sigma_t(2,2) = this->get_parameter("theta_cov").as_double();
    sigma_t(3,3) = this->get_parameter("velocity_cov").as_double();
    sigma_t(4,4) = this->get_parameter("theta_dot_cov").as_double();
    sigma_t(5,5) = this->get_parameter("ax_cov").as_double();
    sigma_t(6,6) = this->get_parameter("ay_cov").as_double();

    //initalize the sensor noice 
    Q_imu.Zero();
    Q_odom.Zero();

    Q_odom(0,0) = this->get_parameter("Q_odom_x").as_double();
    Q_odom(1,1) = this->get_parameter("Q_odom_y").as_double();
    Q_odom(2,2) = this->get_parameter("Q_odom_theta").as_double();
    Q_odom(3,3) = this->get_parameter("Q_odom_v").as_double();
    Q_odom(4,4) = this->get_parameter("Q_odom_theta_dot").as_double();

    Q_imu(0,0) = this->get_parameter("Q_imu_theta").as_double();
    Q_imu(1,1) = this->get_parameter("Q_imu_theta_dot").as_double();
    Q_imu(2,2) = this->get_parameter("Q_imu_ax").as_double();
    Q_imu(3,3) = this->get_parameter("Q_imu_ay").as_double();

    //initalize the process noise
    R.Zero();
    R(0,0) = this->get_parameter("R_x").as_double();
    R(1,1) = this->get_parameter("R_y").as_double();
    R(2,2) = this->get_parameter("R_theta").as_double();
    R(3,3) = this->get_parameter("R_v").as_double();
    R(4,4) = this->get_parameter("R_theta_dot").as_double();
    R(5,5) = this->get_parameter("R_ax").as_double();
    R(6,6) = this->get_parameter("R_ay").as_double();

    //initalize the H jacobian matrix for the sensor models 
    H_imu.Zero();
    H_odom.Zero();

    H_imu(0,2) = 1.0;
    H_imu(1,4) = 1.0;
    H_imu(2,5) = 1.0;
    H_imu(3,6) = 1.0;

    for (int i = 0; i < 5; i ++) {
        H_odom(i,i) = 1.0;
    }

    I7.Identity();

}