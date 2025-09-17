#include "ekf.hpp"

EKF::EKF () : Node ("ekf") {

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

    RCLCPP_INFO(this->get_logger(), "initalized pubs and subs");

    //tf broadcast
    if (should_pub_tf) {
        RCLCPP_INFO(this->get_logger(),"tf is enabled");
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }
}

void EKF::control_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr input) {

    //only calling the init_time() in this one because, we need the prev_input to drive any calculation forwared
    //otherwise, the jacobian calculation corupts and becomes null
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

    prev_input = *input; // save this regardless, a fail. always want the latest control input
    if (current_predition.did_error) {
        //RCLCPP_INFO(this->get_logger(), "control_callback predition corrupt");
        return;
    }

    //save the previous state
    prev_update_time = current_predition.calc_time;
    mu = current_predition.mu;
    sigma_t = current_predition.sigma_t;

    EKF::publish_state();
}

void EKF::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {

    if (!time_init || imu_msg == nullptr) {
        return;
    }

    prediction current_predition = EKF::prediction_step(mu, sigma_t, prev_input);

    //calculate kalman gain
    matrix4d S = (H_imu * current_predition.sigma_t * H_imu.transpose() + Q_imu).eval(); //inovation covariance
    S = (0.5 * (S + S.transpose())).eval(); //correct the matrix scew 

    //note to future self, generalize the kalman gain calculation with a template function and kalman gain class

    //use the llt to try and solve
    Eigen::LLT<matrix4d> llt (S);
    matrix7x4d K;

    if (llt.info() != Eigen::Success) {

        //given llt fails, matrix is not positive definite, add jitter to fix
        // try using ldlt
        S.diagonal().array() += 1e-8;
        S = (0.5 * (S + S.transpose())).eval();

        Eigen::LDLT<matrix4d> ldlt (S);
        RCLCPP_INFO(this->get_logger(), "using ldlt to solve inversion in imu");


        if (ldlt.info() != Eigen::Success) {

            RCLCPP_INFO(this->get_logger(), "faild to safely invert S, discarding this iteration of imu correction");
            return;

        } else {
            K = (current_predition.sigma_t * H_imu.transpose() * ldlt.solve(matrix4d::Identity())).eval();
        }

    } else {
        K = (current_predition.sigma_t * H_imu.transpose() * llt.solve(matrix4d::Identity())).eval();
    }

    vector4d imu_diff = (imu_observation_maker(*imu_msg) - imu_state_mapper(current_predition.mu)).eval();
    vector7d new_mu = (current_predition.mu + K * imu_diff ).eval();

    //josephs formula for new covariance (sigma_t)
    matrix7d A = (matrix7d::Identity() - K * H_imu).eval();
    matrix7d new_sigma_t = (A * current_predition.sigma_t * A.transpose() + K * Q_imu * K.transpose()).eval();

    //final safety check
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

    //normalize the yaw
    new_mu(state_index::theta) = std::atan2(std::sin(new_mu(state_index::theta)), std::cos(new_mu(state_index::theta)));

    //save the results
    mu = new_mu;
    sigma_t = new_sigma_t;
    prev_update_time = current_predition.calc_time;

    //publish the results
    EKF::publish_state();
}

void EKF::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {

    if (!time_init || odom_msg == nullptr) {
        return;
    }

    prediction current_predition = EKF::prediction_step(mu, sigma_t, prev_input);
    
    //calculate kalman gain
    matrix5d S = (H_odom * current_predition.sigma_t * H_odom.transpose() + Q_odom).eval(); //inovation covariance
    S = (0.5 * (S + S.transpose())).eval(); //correct the matrix scew 

    //use the llt to try and solve
    Eigen::LLT<matrix5d> llt (S);
    matrix7x5d K;

    if (llt.info() != Eigen::Success) {

        //given llt fails, matrix is not positive definite, add jitter to fix
        // try using ldlt
        S.diagonal().array() += 1e-8;
        S = (0.5 * (S + S.transpose())).eval();

        Eigen::LDLT<matrix5d> ldlt (S);
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

    //josephs formula for new covariance (sigma_t)
    matrix7d A = (matrix7d::Identity() - K * H_odom).eval();
    matrix7d new_sigma_t = (A * current_predition.sigma_t * A.transpose() + K * Q_odom * K.transpose()).eval();

    //final safety check
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

    //normalize the yaw
    new_mu(state_index::theta) = std::atan2(std::sin(new_mu(state_index::theta)), std::cos(new_mu(state_index::theta)));

    //save the results
    mu = new_mu;
    sigma_t = new_sigma_t;
    prev_update_time = current_predition.calc_time;

    //publish the results
    EKF::publish_state();
}

vector7d EKF::model_update(const vector7d &mu_prev, const ackermann_msgs::msg::AckermannDriveStamped &control_input, double dt_) {

    vector7d predicted_state;
    double v = control_input.drive.speed;

    predicted_state << 
        mu_prev(state_index::x) + v * std::cos(mu_prev(state_index::theta)) * dt_ + 0.5 * mu_prev(state_index::ax) * std::pow(dt_, 2),
        mu_prev(state_index::y) + v * std::sin(mu_prev(state_index::theta)) * dt_ + 0.5 * mu_prev(state_index::ay) * std::pow(dt_, 2),
        mu_prev(state_index::theta) + (v / wheel_base) * std::tan(control_input.drive.steering_angle) * dt_,
        v + mu_prev(state_index::ax) * std::cos(mu_prev(state_index::theta)) * dt_ + mu_prev(state_index::ay) * std::sin(mu_prev(state_index::theta)) * dt_,
        (v / wheel_base) * std::tan(control_input.drive.steering_angle),
        mu_prev(state_index::ax),
        mu_prev(state_index::ay)
    ;

    return predicted_state.eval();
}

matrix7d EKF::predict_sigma(const matrix7d &sigma_prev, const matrix7d &jacobian_g) {
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
            RCLCPP_INFO(this->get_logger(), " %f %f %f %f %f %f %f", jacobian_g(i, 0), jacobian_g(i, 1), jacobian_g(i, 2), jacobian_g(i, 3), jacobian_g(i, 4), jacobian_g(i, 5), jacobian_g(i, 6));
        }
    }

    return sigma_predicted.eval();
}

matrix7d EKF::calc_jacobian_g(const vector7d &mu_prev, double dt_) {

    matrix7d jacobian;
    jacobian.Zero();

    double DT = dt_, v = mu_prev(state_index::v), theta = mu_prev(state_index::theta), phi = prev_input.drive.steering_angle;
    double L_WB = wheel_base, ax = mu_prev(state_index::ax), ay = mu_prev(state_index::ay);
   
    if (debug_mode) {
        RCLCPP_INFO(this->get_logger(), "dt is %f, v is %f, theta is %f, phi is %f, wheel_base is %f, ax is %f, ay is %f", DT, theta, phi, L_WB, ax, ay);
    }

    // 1st row
    jacobian(0, 0) = 1.0;
    jacobian(0, 2) = -v * std::sin(theta) * DT;
    jacobian(0, 3) = std::cos(theta) * DT;
    jacobian(0, 5) = 0.5 * std::pow(DT, 2);

    // 2nd row
    jacobian(1, 1) = 1.0;
    jacobian(1, 2) = v * std::cos(theta) * DT;
    jacobian(1, 3) = std::sin(theta) * DT;
    jacobian(1, 6) = 0.5 * std::pow(DT, 2);

    // 3rd row
    jacobian(2, 2) = 1.0;
    jacobian(2, 3) = (std::tan(phi) * DT) / L_WB;

    // 4th row
    jacobian(3, 2) = ay * std::cos(theta) * DT - ax * std::sin(theta) * DT;
    jacobian(3, 3) = 1.0;
    jacobian(3, 5) = std::cos(theta) * DT;
    jacobian(3, 6) = std::sin(theta) * DT;

    // 5th row
    jacobian(4, 3) = std::tan(phi) / L_WB;

    // 6th row
    jacobian(5, 5) = 1.0;

    // 7th row
    jacobian(6, 6) = 1.0;

    return jacobian.eval();
}

prediction EKF::prediction_step(const vector7d &mu_prev, const matrix7d &sigma_prev, const ackermann_msgs::msg::AckermannDriveStamped &control_input){

    prediction current_prediction;
    
    rclcpp::Time current_time = this->now();
    double dt = EKF::calc_dt(current_time,prev_update_time);

    // don't touch this, if you remove this, the filter stops working
    if (error_counter == 0) {
        RCLCPP_INFO(this->get_logger(), "current dt is %f", dt);
    }

    if (dt <= 1e-6) {
        current_prediction.did_error = true;
        RCLCPP_INFO(this->get_logger(), "dt is too small");
        return current_prediction;
    }

    vector7d mu_bar = EKF::model_update(mu_prev,control_input,dt).eval();
    matrix7d jacobian_G = EKF::calc_jacobian_g(mu_prev,dt).eval();
    matrix7d sigma_bar = EKF::predict_sigma(sigma_prev,jacobian_G).eval();

    //normalize the yaw
    mu_bar(state_index::theta) = std::atan2(std::sin(mu_bar(state_index::theta)), std::cos(mu_bar(state_index::theta)));

    if (!mu_bar.allFinite() || !sigma_bar.allFinite()) {
        current_prediction.did_error = true;
        if (!mu_bar.allFinite()) {
            RCLCPP_INFO(this->get_logger(),"mu_bar prediction is corrupt");
        }
        if (!sigma_bar.allFinite() && error_counter <= 4) {
            RCLCPP_INFO(this->get_logger(),"sigma_bar prediction is corrupt");
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

vector4d EKF::imu_state_mapper(const vector7d &mu_predicted) {

    vector4d observation;
    observation.Zero();

    observation(0) = mu_predicted(state_index::theta);
    observation(1) = mu_predicted(state_index::theta_dot);
    observation(2) = mu_predicted(state_index::ax);
    observation(3) = mu_predicted(state_index::ay);

    return observation.eval();

}

vector5d EKF::odom_state_mapper(const vector7d &mu_predicted) {

    vector5d observation;
    observation.Zero();

    observation(0) = mu_predicted(state_index::x);
    observation(1) = mu_predicted(state_index::y);
    observation(2) = mu_predicted(state_index::theta);
    observation(3) = mu_predicted(state_index::v);
    observation(4) = mu_predicted(state_index::theta_dot);

    return observation.eval();

}

vector4d EKF::imu_observation_maker(const sensor_msgs::msg::Imu &observation) {

    vector4d z;
    z.Zero();

    z(0) = std::atan2(2.0 * (observation.orientation.w * observation.orientation.z +
        observation.orientation.x * observation.orientation.y),
        1.0 - 2.0 * (observation.orientation.y * observation.orientation.y +
        observation.orientation.z * observation.orientation.z));

    z(1) = observation.angular_velocity.z;
    z(2) = observation.linear_acceleration.x;
    z(3) = observation.linear_acceleration.y;

    return z.eval();
}

vector5d EKF::odom_observation_maker(const nav_msgs::msg::Odometry &observation) {

    vector5d z;
    z.Zero();

    z(0) = observation.pose.pose.position.x;
    z(1) = observation.pose.pose.position.y;
    z(2) = std::atan2(2.0 * (observation.pose.pose.orientation.w * observation.pose.pose.orientation.z + 
        observation.pose.pose.orientation.x * observation.pose.pose.orientation.y),
        1.0 - 2.0 * (observation.pose.pose.orientation.y * observation.pose.pose.orientation.y +
        observation.pose.pose.orientation.z * observation.pose.pose.orientation.z));
    
    z(3) = observation.twist.twist.linear.x;
    z(4) = observation.twist.twist.angular.z;

    return z.eval();

}
double EKF::calc_dt(rclcpp::Time &current, rclcpp::Time &prev) {
    return static_cast<double>((current.seconds() + current.nanoseconds() * 1e-9) - (prev.seconds() + prev.nanoseconds() * 1e-9));
}

void EKF::init_time() {
    prev_update_time = this->now();
    time_init = true;
}

void EKF::publish_state() {
    EKF::publish_odom();

    if (should_pub_tf) {
        EKF::publish_tf();
    }

}

void EKF::publish_odom() {

    nav_msgs::msg::Odometry ekf_msg;

    //lowpass filtering
    double lowpass_speed = EKF::velocity_lowPass(mu(state_index::v));
    mu(state_index::v) = lowpass_speed;

    ekf_msg.child_frame_id = child_frame;
    ekf_msg.header.frame_id = header_frame;
    ekf_msg.header.stamp = prev_update_time;

    tf2::Quaternion q;
    q.setRPY(0,0,mu(state_index::theta));

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
            RCLCPP_INFO(this->get_logger(), " %f %f %f %f %f %f %f", sigma_t(i, 0), sigma_t(i, 1), sigma_t(i, 2), sigma_t(i, 3), sigma_t(i, 4), sigma_t(i, 5), sigma_t(i, 6));
        }
    }

    //bundle in the covariance with the mesurments

    //pose covariance 
    ekf_msg.pose.covariance.at(0) = sigma_t(0,0); // x
    ekf_msg.pose.covariance.at(7) = sigma_t(1,1); // y
    ekf_msg.pose.covariance.at(35) = sigma_t(2,2); // yaw

    ekf_msg.twist.covariance.at(0) = sigma_t(3,3); // v
    ekf_msg.twist.covariance.at(35) = sigma_t(4,4); // thata_dot or angular velocity about z axis

    ekf_pub->publish(ekf_msg);
}

void EKF::publish_tf () {
    
    geometry_msgs::msg::TransformStamped t;

    t.child_frame_id = child_frame;
    t.header.frame_id = header_frame;
    t.header.stamp = prev_update_time;

    t.transform.translation.x = mu(state_index::x);
    t.transform.translation.y = mu(state_index::y);
    t.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0,0,mu(state_index::theta));

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    
    tf_broadcaster->sendTransform(t);
}

double EKF::velocity_lowPass(double current_speed) {

    double lowpass_speed = current_speed * alpha + (1 - alpha) * prev_speed;
    prev_speed = lowpass_speed;

    return lowpass_speed;

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
    this->declare_parameter<bool>("publish_tf",true);

    //physicl quantity declaration
    this->declare_parameter<double>("wheel_base",0.3240);

    //low pass filter
    this->declare_parameter<double>("alpha", 0.40);

    //debug mode
    this->declare_parameter<bool>("debug_mode", false);

    //mu inital declaration
    this->declare_parameter<double>("inital_x",0.7412);
    this->declare_parameter<double>("inital_y",3.1583);
    this->declare_parameter<double>("inital_theta",-M_PI/1);
    this->declare_parameter<double>("inital_velocity", 0.0);
    this->declare_parameter<double>("inital_theta_dot", 0.0);
    this->declare_parameter<double>("inital_ax", 0.0);
    this->declare_parameter<double>("inital_ay", 0.0);

    //inital covariance matrix declaration
    this->declare_parameter<double>("x_cov",10.0);
    this->declare_parameter<double>("y_cov",10.0);
    this->declare_parameter<double>("theta_cov",10.0);
    this->declare_parameter<double>("velocity_cov", 10.1);
    this->declare_parameter<double>("theta_dot_cov", 10.0);
    this->declare_parameter<double>("ax_cov", 10.0);
    this->declare_parameter<double>("ay_cov", 10.0);

    //declare the process noise matrix 
    this->declare_parameter<double>("R_x",1.0);
    this->declare_parameter<double>("R_y",1.0);
    this->declare_parameter<double>("R_theta",5.0);
    this->declare_parameter<double>("R_v",1.0);
    this->declare_parameter<double>("R_theta_dot",5.0);
    this->declare_parameter<double>("R_ax",3.50);
    this->declare_parameter<double>("R_ay",3.50);

    //declare the sensor noise matrix
    this->declare_parameter<double>("Q_odom_x",1.5);
    this->declare_parameter<double>("Q_odom_y",1.5);
    this->declare_parameter<double>("Q_odom_theta",3.0);
    this->declare_parameter<double>("Q_odom_v",0.75);
    this->declare_parameter<double>("Q_odom_theta_dot",3.0);

    this->declare_parameter<double>("Q_imu_theta",0.45);
    this->declare_parameter<double>("Q_imu_theta_dot",0.5);
    this->declare_parameter<double>("Q_imu_ax",0.65);
    this->declare_parameter<double>("Q_imu_ay",0.65);

    //topic retrival
    odom_topic = this->get_parameter("odom_topic").as_string();
    ekf_topic = this->get_parameter("ekf_topic").as_string();
    imu_topic = this->get_parameter("imu_topic").as_string();
    ackermann_topic = this->get_parameter("ackermann_topic").as_string();
    should_pub_tf = this->get_parameter("publish_tf").as_bool();

    //frame retrival
    child_frame = this->get_parameter("child_frame").as_string();
    header_frame = this->get_parameter("header_frame").as_string();

    //pysical quantity retrival
    wheel_base = this->get_parameter("wheel_base").as_double();

    //low pass filter
    alpha = this->get_parameter("alpha").as_double();

    //debug mode 
    debug_mode = this->get_parameter("debug_mode").as_bool();

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

    RCLCPP_INFO(this->get_logger(),"initalized the params");

}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKF>());
  rclcpp::shutdown();
  return 0;
}