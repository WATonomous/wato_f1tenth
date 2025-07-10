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

    //model update 
    double dt = EKF_RT::calculate_delta_t();
    vector7d mu_predicted = EKF_RT::model_update(mu,steering->data, throtel->data,dt);
    matrix7d G = EKF_RT::jacobian_g_update(mu,steering->data,throtel->data,dt);


    //store the previous control inputs

    //publish state

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


double EKF_RT::calculate_delta_t() {

    rclcpp::Time current_time = this->now();
    double dt = (current_time.seconds() + current_time.nanoseconds() * 1e-9) +  
                    (prev_update_time.seconds() + prev_update_time.nanoseconds() * 1e-9);

    prev_update_time = current_time;

    return dt;

}

vector7d EKF_RT::model_update(const vector7d &mu_prev, const double &steering_input, const double &throtel_input, double &dt) {

}

matrix7d EKF_RT::jacobian_g_update (const vector7d &mu_prev, const double &steering_input, const double &throtel_input, double &dt) {

}

matrix7d EKF_RT::predict_sigma (const matrix7d &sigma_prev, const matrix7d &jacobian_g) {

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

}