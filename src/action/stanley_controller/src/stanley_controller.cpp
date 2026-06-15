
#include "stanley_controller.hpp"

Stanley_Controller_Node::Stanley_Controller_Node() : Node("stanley_controller_node") {

    //parameters
    Stanley_Controller_Node::init_parameters();

    //publisher
    controls_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        ackermann_control_topic, 10);

    //subscriptions
    auto latched_qos = rclcpp::QoS(1).transient_local().reliable();

    dead_man_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        dead_man_active_topic, latched_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            dead_man_active = *msg;
        }
    );

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        global_path_topic, latched_qos,
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
            current_global_path = *msg;
        }
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_pose = *msg;
        }
    );

    speed_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        speed_topic, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_velocity = msg->twist.twist.linear.x;
        }
    );

    control_loop_timer = this->create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() {
            control_timer_callback();
        }
    );

}

void Stanley_Controller_Node::control_timer_callback() {

    //update controller state
    Stanley_Controller_Node::update_controller_state();

    //don't apply any control if the controller is not active, stop everything
    if (controller_state == stanley_state_::INACTIVE) {
        RCLCPP_WARN(this->get_logger(), "Dead Man switch is off");
        controls_pub_->publish(Stanley_Controller_Node::dead_stop());
        return;
    }

    if (current_global_path.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "no waypoints in global path while in GLOBAL_FOLLOW state");
        controls_pub_->publish(Stanley_Controller_Node::dead_stop());
        return;
    }

    //apply the stanley control law
    ackermann_msgs::msg::AckermannDriveStamped control_command = Stanley_Controller_Node::calculate_control();

    //publish control inputs
    controls_pub_->publish(control_command);

}

void Stanley_Controller_Node::update_controller_state() {

    if (dead_man_active.data) {
        controller_state = stanley_state_::GLOBAL_FOLLOW;
    } else {
        controller_state = stanley_state_::INACTIVE;
    }

}

ackermann_msgs::msg::AckermannDriveStamped Stanley_Controller_Node::calculate_control() {

    //get current pose
    double cx = current_pose.pose.pose.position.x;
    double cy = current_pose.pose.pose.position.y;
    double yaw = Stanley_Controller_Node::extractYaw(current_pose.pose.pose.orientation);

    //calculate front axle position
    double front_x = cx + wheelbase * std::cos(yaw);
    double front_y = cy + wheelbase * std::sin(yaw);

    //find closest waypoint to front axle
    size_t closest_idx = Stanley_Controller_Node::find_closest_point(front_x, front_y);
    size_t next_idx = (closest_idx + 1) % current_global_path.poses.size();

    //path heading at closest point
    double path_dx = current_global_path.poses[next_idx].pose.position.x -
                     current_global_path.poses[closest_idx].pose.position.x;
    double path_dy = current_global_path.poses[next_idx].pose.position.y -
                     current_global_path.poses[closest_idx].pose.position.y;
    double path_heading = std::atan2(path_dy, path_dx);

    //current heading of car
    double current_heading = yaw;
    if (current_heading < 0) current_heading += 2 * M_PI;
    if (path_heading < 0) path_heading += 2 * M_PI;

    //cross track error in car frame
    double dx = current_global_path.poses[closest_idx].pose.position.x - front_x;
    double dy = current_global_path.poses[closest_idx].pose.position.y - front_y;
    double cross_track_error = std::cos(yaw) * dy - std::sin(yaw) * dx;

    //cross track correction
    double cross_track_correction = std::atan2(k_e * cross_track_error, current_velocity + 1e-6);

    //heading error
    double heading_error = path_heading - current_heading;
    if (heading_error > M_PI) heading_error -= 2 * M_PI;
    if (heading_error < -M_PI) heading_error += 2 * M_PI;
    heading_error *= k_h;

    //stanley formula
    double steering_angle = heading_error + cross_track_correction;
    steering_angle = std::clamp(steering_angle, -max_steering_angle, max_steering_angle);

    //get target speed from path z value
    double target_speed = current_global_path.poses[closest_idx].pose.position.z;
    if (speed_limit_enable && target_speed > speed_limit) {
        target_speed = speed_limit;
    }

    RCLCPP_INFO(this->get_logger(),
        "heading_err=%.3f, cte=%.3f, steer=%.3f, speed=%.2f",
        heading_error, cross_track_error, steering_angle, target_speed);

    ackermann_msgs::msg::AckermannDrive drive;
    drive.steering_angle = steering_angle;
    drive.speed = target_speed;

    ackermann_msgs::msg::AckermannDriveStamped stamp;
    stamp.drive = drive;
    stamp.header.frame_id = "base_link";
    stamp.header.stamp = this->now();

    return stamp;

}

size_t Stanley_Controller_Node::find_closest_point(double x, double y) {

    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < current_global_path.poses.size(); i++) {
        double dx = x - current_global_path.poses[i].pose.position.x;
        double dy = y - current_global_path.poses[i].pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    return closest_idx;

}

double Stanley_Controller_Node::extractYaw(const geometry_msgs::msg::Quaternion &quat) {

    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;

}

ackermann_msgs::msg::AckermannDriveStamped Stanley_Controller_Node::dead_stop() {

    ackermann_msgs::msg::AckermannDrive drive;
    drive.speed = 0.0;
    drive.steering_angle = 0.0;

    ackermann_msgs::msg::AckermannDriveStamped stamp;
    stamp.drive = drive;
    stamp.header.frame_id = "base_link";
    stamp.header.stamp = this->now();

    return stamp;

}

void Stanley_Controller_Node::init_parameters() {

    //declare parameters
    this->declare_parameter<std::string>("global_path_topic", "/global_planner/path");
    this->declare_parameter<std::string>("dead_man_active_topic", "/dead_man_switch");
    this->declare_parameter<std::string>("ackermann_control_topic", "/drive/autonomy");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("speed_topic", "/autodrive/roboracer_1/odom");

    this->declare_parameter<bool>("speed_limit_active", true);
    this->declare_parameter<double>("speed_limit", 3.0);
    this->declare_parameter<double>("max_steering_angle", 0.52);
    this->declare_parameter<double>("k_e", 0.2);
    this->declare_parameter<double>("k_h", 1.0);
    this->declare_parameter<double>("wheelbase", 0.3);

    //init parameters
    global_path_topic = this->get_parameter("global_path_topic").as_string();
    dead_man_active_topic = this->get_parameter("dead_man_active_topic").as_string();
    ackermann_control_topic = this->get_parameter("ackermann_control_topic").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    speed_topic = this->get_parameter("speed_topic").as_string();

    speed_limit_enable = this->get_parameter("speed_limit_active").as_bool();
    speed_limit = this->get_parameter("speed_limit").as_double();
    max_steering_angle = this->get_parameter("max_steering_angle").as_double();
    k_e = this->get_parameter("k_e").as_double();
    k_h = this->get_parameter("k_h").as_double();
    wheelbase = this->get_parameter("wheelbase").as_double();

    //initialize state and internal variables
    dead_man_active.data = false;
    controller_state = stanley_state_::INACTIVE;
    current_velocity = 0.0;

}

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Stanley_Controller_Node>());
    rclcpp::shutdown();
    return 0;

}