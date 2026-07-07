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

   if (enable_debug_vis) {
    publish_debug_vis(current_pose.pose.pose,
                      last_closest_idx_,
                      last_cross_track_error_,
                      last_heading_error_,
                      last_heading_term_,
                      last_cte_term_,
                      last_steering_cmd_,
                      current_velocity);
}
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
    
/*
    //heading error
    double heading_error = path_heading - current_heading;
    if (heading_error > M_PI) heading_error -= 2 * M_PI;
    if (heading_error < -M_PI) heading_error += 2 * M_PI;
    heading_error *= k_h;

    //stanley formula
    double steering_angle = heading_error + cross_track_correction;
    steering_angle = std::clamp(steering_angle, -max_steering_angle, max_steering_angle);
*/
    //heading error (raw, in [-pi, pi])
    double heading_error = path_heading - current_heading;
    if (heading_error > M_PI) heading_error -= 2 * M_PI;
    if (heading_error < -M_PI) heading_error += 2 * M_PI;

    //stanley terms 
    double heading_term = k_h * heading_error;
    double cte_term     = cross_track_correction;
    double steering_angle = heading_term + cte_term;
    steering_angle = std::clamp(steering_angle, -max_steering_angle, max_steering_angle);

    //stash for debug viz
    last_closest_idx_       = closest_idx;
    last_cross_track_error_ = cross_track_error;
    last_heading_error_     = heading_error;
    last_heading_term_      = heading_term;
    last_cte_term_          = cte_term;
    last_steering_cmd_      = steering_angle;

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
    this->declare_parameter<double>("k_h", 0.75);
    this->declare_parameter<double>("wheelbase", 0.324);

    this->declare_parameter<bool>("enable_debug_vis", true);

    this->declare_parameter<std::string>("global_frame_id", "map");
    this->declare_parameter<std::string>("local_frame_id", "base_link");

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

    enable_debug_vis = this->get_parameter("enable_debug_vis").as_bool();

    debug_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/debug/stanley_markers", 10);
    cte_pub_ = this->create_publisher<std_msgs::msg::Float32>("/debug/stanley/cte", 10);
    heading_err_pub_ = this->create_publisher<std_msgs::msg::Float32>("/debug/stanley/heading_error", 10);
    heading_term_pub_ = this->create_publisher<std_msgs::msg::Float32>("/debug/stanley/heading_term", 10);
    cte_term_pub_ = this->create_publisher<std_msgs::msg::Float32>("/debug/stanley/cte_term", 10);
    delta_pub_ = this->create_publisher<std_msgs::msg::Float32>("/debug/stanley/delta_cmd", 10);

    global_frame_id = this->get_parameter("global_frame_id").as_string();
    local_frame_id  = this->get_parameter("local_frame_id").as_string();

    //initialize state and internal variables
    dead_man_active.data = false;
    controller_state = stanley_state_::INACTIVE;
    current_velocity = 0.0;

}

void Stanley_Controller_Node::publish_debug_vis(const geometry_msgs::msg::Pose& base_link_pose,
                                     size_t closest_idx,
                                     double cross_track_error,
                                     double heading_error,
                                     double heading_term,
                                     double cte_term,
                                     double steering_cmd,
                                     double velocity) {

    auto stamp = this->now();

    // --- Scalar publishers for time-series plotting (Foxglove / PlotJuggler) ---
    std_msgs::msg::Float32 f;
    f.data = cross_track_error; cte_pub_->publish(f);
    f.data = heading_error;     heading_err_pub_->publish(f);
    f.data = heading_term;      heading_term_pub_->publish(f);
    f.data = cte_term;          cte_term_pub_->publish(f);
    f.data = steering_cmd;      delta_pub_->publish(f);

    // --- Geometry: front axle, closest point, path tangent ---
    const double yaw = std::atan2(
        2.0 * (base_link_pose.orientation.w * base_link_pose.orientation.z +
               base_link_pose.orientation.x * base_link_pose.orientation.y),
        1.0 - 2.0 * (std::pow(base_link_pose.orientation.y, 2) +
                     std::pow(base_link_pose.orientation.z, 2)));

    const double fx = base_link_pose.position.x + wheelbase * std::cos(yaw);
    const double fy = base_link_pose.position.y + wheelbase * std::sin(yaw);

    double cx = 0, cy = 0, tx = 1, ty = 0;
    const auto& poses = current_global_path.poses;
    if (closest_idx < poses.size()) {
        cx = poses[closest_idx].pose.position.x;
        cy = poses[closest_idx].pose.position.y;
        size_t next = std::min(closest_idx + 1, poses.size() - 1);
        if (next == closest_idx && closest_idx > 0) {
            tx = poses[closest_idx].pose.position.x - poses[closest_idx-1].pose.position.x;
            ty = poses[closest_idx].pose.position.y - poses[closest_idx-1].pose.position.y;
        } else {
            tx = poses[next].pose.position.x - cx;
            ty = poses[next].pose.position.y - cy;
        }
        double n = std::hypot(tx, ty);
        if (n > 1e-6) { tx /= n; ty /= n; }
    }

    // --- MarkerArray ---
    visualization_msgs::msg::MarkerArray arr;
    auto make_header = [&](visualization_msgs::msg::Marker& m,
                           const std::string& ns, int id, int32_t type) {
        m.header.frame_id = global_frame_id;
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = id;
        m.type = type;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
    };

    // 1. Reference path
    {
        visualization_msgs::msg::Marker m;
        make_header(m, "path", 0, visualization_msgs::msg::Marker::LINE_STRIP);
        m.scale.x = 0.05;
        m.color.r = 0.4; m.color.g = 0.4; m.color.b = 0.4; m.color.a = 0.8;
        for (auto& p : poses) {
            geometry_msgs::msg::Point pt;
            pt.x = p.pose.position.x; pt.y = p.pose.position.y; pt.z = 0.02;
            m.points.push_back(pt);
        }
        arr.markers.push_back(m);
    }

    // 2. Front axle 
    {
        visualization_msgs::msg::Marker m;
        make_header(m, "front_axle", 0, visualization_msgs::msg::Marker::SPHERE);
        m.pose.position.x = fx; m.pose.position.y = fy; m.pose.position.z = 0.1;
        m.scale.x = m.scale.y = m.scale.z = 0.15;
        m.color.r = 1.0; m.color.g = 1.0; m.color.a = 1.0;
        arr.markers.push_back(m);
    }

    // 3. Closest path point 
    {
        visualization_msgs::msg::Marker m;
        make_header(m, "closest_point", 0, visualization_msgs::msg::Marker::SPHERE);
        m.pose.position.x = cx; m.pose.position.y = cy; m.pose.position.z = 0.1;
        m.scale.x = m.scale.y = m.scale.z = 0.18;
        m.color.g = 1.0; m.color.a = 1.0;
        arr.markers.push_back(m);
    }

    // 4. CTE line 
    {
        visualization_msgs::msg::Marker m;
        make_header(m, "cte", 0, visualization_msgs::msg::Marker::LINE_STRIP);
        m.scale.x = 0.04;
        double mag = std::min(1.0, std::abs(cross_track_error) / 0.5);
        if (cross_track_error >= 0) m.color.b = 1.0;
        else                        m.color.r = 1.0;
        m.color.g = 1.0 - mag;
        m.color.a = 1.0;
        geometry_msgs::msg::Point a, b;
        a.x = fx; a.y = fy; a.z = 0.1;
        b.x = cx; b.y = cy; b.z = 0.1;
        m.points = {a, b};
        arr.markers.push_back(m);
    }

    // 5. Path tangent arrow 
    {
        visualization_msgs::msg::Marker m;
        make_header(m, "path_tangent", 0, visualization_msgs::msg::Marker::ARROW);
        geometry_msgs::msg::Point a, b;
        a.x = cx; a.y = cy; a.z = 0.15;
        b.x = cx + 0.6*tx; b.y = cy + 0.6*ty; b.z = 0.15;
        m.points = {a, b};
        m.scale.x = 0.04; m.scale.y = 0.08; m.scale.z = 0.1;
        m.color.g = 1.0; m.color.b = 0.3; m.color.a = 1.0;
        arr.markers.push_back(m);
    }

    // 6. Vehicle heading arrow 
    {
        visualization_msgs::msg::Marker m;
        make_header(m, "vehicle_heading", 0, visualization_msgs::msg::Marker::ARROW);
        geometry_msgs::msg::Point a, b;
        a.x = fx; a.y = fy; a.z = 0.15;
        b.x = fx + 0.6*std::cos(yaw); b.y = fy + 0.6*std::sin(yaw); b.z = 0.15;
        m.points = {a, b};
        m.scale.x = 0.04; m.scale.y = 0.08; m.scale.z = 0.1;
        m.color.r = 1.0; m.color.g = 0.5; m.color.a = 1.0;
        arr.markers.push_back(m);
    }

    // 7. Steering command arrow 
    {
        visualization_msgs::msg::Marker m;
        make_header(m, "steering_cmd", 0, visualization_msgs::msg::Marker::ARROW);
        double steer_dir = yaw + steering_cmd;
        geometry_msgs::msg::Point a, b;
        a.x = fx; a.y = fy; a.z = 0.2;
        b.x = fx + 0.5*std::cos(steer_dir); b.y = fy + 0.5*std::sin(steer_dir); b.z = 0.2;
        m.points = {a, b};
        m.scale.x = 0.05; m.scale.y = 0.1; m.scale.z = 0.12;
        m.color.r = 1.0; m.color.b = 1.0; m.color.a = 1.0;
        arr.markers.push_back(m);
    }

    // 8. Debug text above the car
    {
        visualization_msgs::msg::Marker m;
        make_header(m, "debug_text", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        m.pose.position.x = base_link_pose.position.x;
        m.pose.position.y = base_link_pose.position.y;
        m.pose.position.z = 0.6;
        m.scale.z = 0.18;
        m.color.r = m.color.g = m.color.b = 1.0; m.color.a = 1.0;
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "cte=%+.2fm\ndelta=%+.1fdeg\nv=%.2fm/s idx=%zu",
                      cross_track_error,
                      steering_cmd * 180.0 / M_PI,
                      velocity, closest_idx);
        m.text = buf;
        arr.markers.push_back(m);
    }

    debug_markers_pub_->publish(arr);
}

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Stanley_Controller_Node>());
    rclcpp::shutdown();
    return 0;

}