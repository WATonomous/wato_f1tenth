#include "ekf_rt.hpp"

EKF_RT::EKF_RT () : Node ("ekf_rt") {

    //parameters  
    EKF_RT:initalize_params();

    //pubs and subs
    steering_sub.subscribe(this,steering_topic);
    throtel_sub.subscribe(this,throtel_topic);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), throtel_sub, steering_sub);
    sync_->registerCallback(std::bind(EKF_RT::control_input_callback, this , std::placeholders::_1,std::placeholders::_2));

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic,10,std::bind(EKF_RT::imu_callback,this,std::placeholders::_1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic,10,std::bind(EKF_RT::odom_callback,this,std::placeholders::_1));

}

void EKF_RT::control_input_callback(std_msgs::msg::Float32::SharedPtr throtel,std_msgs::msg::Float32::SharedPtr steering) {

    if (!init_time) {
        EKF_RT::is_time_init();
        return;
    }

    //  predition step
    rclcpp::Time current_time = this->now();
    double dt = EKF_RT::calculate_delta_t(current_time, prev_update_time);
    vector7d mu_predicted = EKF_RT::model_update(mu, steering->data, throtel->data, dt);
    matrix7d G = EKF_RT::jacobian_g_update(mu, steering->data, throtel->data, dt);
    matrix7d sigma_predicted = EKF_RT::predict_sigma(sigma_t, G);

    bool store_prev_state = true;

    // check the valicity of the answer
    if (!mu_predicted.allFinite() || !sigma_predicted.allFinite()) {
        RCLCPP_INFO(this->get_logger(), "mu or sigma has corrupted after calculations");
        store_prev_state = false;
    }

    // store prv state and control inputs
    prev_steering = *steering;
    prev_throtel = *throtel;
    
    if (!store_prev_state) {
        return;
    }

    mu = mu_predicted;
    sigma_t = sigma_predicted;
    prev_update_time = current_time;

    // publish state
    EKF_RT::publish_state();

}

void EKF_RT::imu_callback (sensor_msgs::msg::Imu::SharedPtr msg) {
    
    if (!init_time) {
        EKF_RT::is_time_init();
        return;
    }
    

    //prediction step

    //correction step with imu data

    //publish data
}

void EKF_RT::odom_callback (nav_msgs::msg::Odometry::SharedPtr msg) {
    
    if (!init_time) {
        EKF_RT::is_time_init();
        return;
    }

    //prediction step

    //correction step with odom data

    //publish data
}

void EKF_RT::is_time_init () {
    prev_update_time = this->now();
    init_time = true;
}

//remember only calcuates the curret delta, dosen't store the previous values
//do not ever make this function store the previous value, only store it after 
//confirming that the values from the calcuation is not nan
double EKF_RT::calculate_delta_t(rclcpp::Time current, rclcpp::Time prev) {

    double dt = (current.seconds() + current.nanoseconds() * 1e-9) + (prev.seconds() + prev.nanoseconds() * 1e-9);

    return dt;

}

vector7d EKF_RT::model_update(const vector7d &mu_prev, const double &steering_input, const double &throtel_input, double &dt) {

    double v = throtel_input * max_speed;

    vector7d predicted_state;

    predicted_state << 
        mu_prev(state::x) + v * std::cos(mu_prev(state::theta)) * dt + 0.5 * mu_prev(state::ax) * std::pow(dt, 2),
        mu_prev(state::y) + v * std::sin(mu_prev(state::theta)) * dt + 0.5 * mu_prev(state::ay) * std::pow(dt, 2),
        theta + (v / wheel_base) * std::tan(steering_input) * dt,
        v + mu_prev(state::ax) * std::cos(mu_prev(state::theta)) * dt + mu_prev(state::ay) * std::sin(mu_prev(state::theta)) * dt,
        (v / wheel_base) * std::tan(steering_input),
        mu_prev(state::ax),
        mu_prev(state::ay)
    ;

    return predicted_state;
}

matrix7d EKF_RT::jacobian_g_update (const vector7d &mu_prev, const double &steering_input, const double &throtel_input, double &dt) {

    double v = throtel_input * max_speed;

    matrix7d jacobian;
    jacobian.Zero();

    // 1st row
    jacobian(0, 0) = 1.0;
    jacobian(0, 2) = -v * std::sin(mu_prev(state::theta)) * dt;
    jacobian(0, 3) = std::cos(mu_prev(state::theta)) * dt;
    jacobian(0, 5) = 0.5 * std::pow(dt, 2);

    // 2nd row
    jacobian(1, 1) = 1.0;
    jacobian(1, 2) = v * std::cos(mu_prev(state::theta)) * dt;
    jacobian(1, 3) = std::sin(mu_prev(state::theta)) * dt;
    jacobian(1, 6) = 0.5 * std::pow(dt, 2);

    // 3rd row
    jacobian(2, 2) = 1.0;
    jacobian(2, 3) = (std::tan(steering_input) * dt) / wheel_base;

    // 4th row
    jacobian(3, 2) = mu_prev(state::ay) * std::cos(mu_prev(state::theta)) * dt - mu_prev(state::ax) * std::sin(mu_prev(state::theta)) * dt;
    jacobian(3, 3) = 1.0;
    jacobian(3, 5) = std::cos(mu_prev(state::theta)) * dt;
    jacobian(3, 6) = std::sin(mu_prev(state::theta)) * dt;

    // 5th row
    jacobian(4, 3) = std::tan(steering_input) / wheel_base;

    // 6th row
    jacobian(5, 5) = 1.0;

    // 7th row
    jacobian(6, 6) = 1.0;

    return jacobian;
}

matrix7d EKF_RT::predict_sigma (const matrix7d &sigma_prev, const matrix7d &jacobian_g) {
    matrix7d sigma_predicted = jacobian_g * sigma_predicted * jacobian_g.transpose() + R;
    return sigma_predicted;
}

void EKF_RT::publish_state() {

    nav_msgs::msg::Odometry ekf_msg;

    ekf_msg.child_frame_id = child_frame;
    ekf_msg.header.frame_id = header_frame;
    ekf_msg.header.stamp = prev_update_time;

    tf2::Quaternion q;
    q.setRPY(0,0,mu(state::theta));

    ekf_msg.pose.pose.position.x = mu(state::x);
    ekf_msg.pose.pose.position.y = mu(state::y);

    ekf_msg.pose.pose.orientation.x = q.x();
    ekf_msg.pose.pose.orientation.y = q.y();
    ekf_msg.pose.pose.orientation.z = q.z();
    ekf_msg.pose.pose.orientation.w = q.w();

    ekf_msg.twist.twist.linear.x = mu(state::v);
    ekf_msg.twist.twist.angular.z = mu(state::theta_dot);

    ekf_odom_pub->publish(ekf_msg);

}

void EKF_RT::initalize_params () {

    //topic decliration
    this->declare_parameter<std::string>("ekf_topic","/ekf/odom");
    this->declare_parameter<std::string>("odom_topic","/odom");
    this->declare_parameter<std::string>("steering_topic","/autodrive/f1tenth_1/steering");
    this->declare_parameter<std::string>("throtel_topic","/autodrive/f1tenth_1/throtel");
    this->declare_parameter<std::string>("imu_topic","/autodrive/f1tenth_1/imu");

    //frames declaration
    this->declare_parameter<std::string>("child_frame","base_link");
    this->declare_parameter<std::string>("header_frame","odom");

    //physicl quantity declaration
    this->declare_parameter<double>("wheel_base",0.3240);
    this->declare_parameter<double>("max_speed",22.88);

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
    this->declare_parameter<double>("Q_x",0.1);
    this->declare_parameter<double>("Q_y",0.1);
    this->declare_parameter<double>("Q_theta",0.1);
    this->declare_parameter<double>("Q_v",0.1);
    this->declare_parameter<double>("Q_theta_dot",0.1);
    this->declare_parameter<double>("Q_ax",0.1);
    this->declare_parameter<double>("Q_ay",0.1);


    //topic retrival
    odom_topic = this->get_parameter("odom_topic").as_string();
    ekf_topic = this->get_parameter("ekf_topic").as_string();
    imu_topic = this->get_parameter("imu_topic").as_string();
    throtel_topic  = this->get_parameter("throtel_topic").as_string();
    steering_topic = this->get_parameter("steering_topic").as_string();

    //frame retrival
    child_frame = this->get_parameter("child_frame").as_string();
    header_frame = this->get_parameter("header_frame").as_string();

    //pysical quantity retrival
    wheel_base = this->get_parameter("wheel_base").as_double();
    max_speed = this->get_parameter("max_speed").as_double();

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

}