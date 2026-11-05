#include "stanley_controller.hpp"

Stanley_Controller_Node::Stanley_Controller_Node() : Node("stanley_controller_node") {

    init_parameters();

    controls_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        ackermann_control_topic, 10);

    debug_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/debug/stanley/markers", rclcpp::SensorDataQoS());

    debug_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/debug/stanley/path", rclcpp::QoS(1).transient_local());
    cte_pub_          = this->create_publisher<std_msgs::msg::Float32>("/debug/stanley/cte", rclcpp::SensorDataQoS());
    heading_err_pub_  = this->create_publisher<std_msgs::msg::Float32>("/debug/stanley/heading_error", rclcpp::SensorDataQoS());
    heading_term_pub_ = this->create_publisher<std_msgs::msg::Float32>("/debug/stanley/heading_term", rclcpp::SensorDataQoS());
    cte_term_pub_     = this->create_publisher<std_msgs::msg::Float32>("/debug/stanley/cte_term", rclcpp::SensorDataQoS());
    delta_pub_        = this->create_publisher<std_msgs::msg::Float32>("/debug/stanley/delta_cmd", rclcpp::SensorDataQoS());

    //subscriptions
    //volatile + reliable matches both the gamepad's latched publisher and plain
    //volatile ones (Foxglove, ros2 topic pub); transient_local would ignore those
    auto dead_man_qos = rclcpp::QoS(1).reliable();
    auto latched_qos = rclcpp::QoS(1).transient_local().reliable();

    dead_man_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        dead_man_active_topic, dead_man_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            dead_man_active = *msg;
        }
    );

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        global_path_topic, latched_qos,
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
            current_global_path = *msg;
            stanley::drop_duplicate_points(current_global_path.poses);

            //new path invalidates the cached search index
            closest_idx_initialized_ = false;
            if (enable_debug_vis) {
                publish_debug_path();
            }
        }
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose = *msg;
        last_pose_rx_ = this->now();
    }
);
    speed_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    speed_topic, 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_velocity = msg->twist.twist.linear.x;
    }
);
    control_loop_timer = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_hz),
        [this]() {
            control_timer_callback();
        }
    );
}

void Stanley_Controller_Node::control_timer_callback() {

    update_controller_state();

    const bool have_path = !current_global_path.poses.empty();

    const bool pose_fresh = last_pose_rx_.nanoseconds() != 0 &&
                            (this->now() - last_pose_rx_).seconds() <= pose_timeout_s;

    if (controller_state == stanley_state_::INACTIVE) {

        //releasing the dead man switch is the only way to clear a pose-lost stop
        pose_lost_ = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Dead Man switch is off");
        controls_pub_->publish(dead_stop());

    } else if (!have_path) {

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "no waypoints in global path while in GLOBAL_FOLLOW state");
        controls_pub_->publish(dead_stop());

    } else if (!pose_fresh || pose_lost_) {

        //no pose yet, or localization stalled while armed: stop, and stay stopped until the dead man switch is released
        if (!pose_fresh) {
            pose_lost_ = true;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "no fresh pose on %s, holding the car stopped", odom_topic.c_str());
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "pose is back, release and re-press the dead man switch to drive");
        }
        controls_pub_->publish(dead_stop());

    } else {

        //apply the stanley control law
        controls_pub_->publish(calculate_control());

    }

    //publish viz whether or not we're actively driving
    if (enable_debug_vis && have_path && tick_++ % vis_every_n_ == 0) {
        publish_debug_vis(current_pose.pose.pose,
                          last_closest_idx_,
                          last_cross_track_error_,
                          last_heading_error_,
                          last_heading_term_,
                          last_cte_term_,
                          last_steering_cmd_,
                          current_velocity);
    }

}

void Stanley_Controller_Node::update_controller_state() {

    if (dead_man_active.data) {
        controller_state = stanley_state_::GLOBAL_FOLLOW;
    }

    else {
        controller_state = stanley_state_::INACTIVE;
    }

}

ackermann_msgs::msg::AckermannDriveStamped Stanley_Controller_Node::calculate_control() {

    //current pose and speed
    stanley::State s;
    s.x     = current_pose.pose.pose.position.x;
    s.y     = current_pose.pose.pose.position.y;
    s.yaw   = extractYaw(current_pose.pose.pose.orientation);
    s.speed = current_velocity;

    stanley::Params p;
    p.k_h                = k_h;
    p.k_e                = k_e;
    p.k_soft             = k_soft;
    p.max_steering_angle = max_steering_angle;
    p.wheelbase          = wheelbase;
    p.speed_limit_active = speed_limit_enable;
    p.speed_limit        = speed_limit;

    //find closest waypoint to front axle, then apply the control law (stanley.cpp)
    double front_x, front_y;
    stanley::front_axle(s, wheelbase, front_x, front_y);
    size_t closest_idx = find_closest_point(front_x, front_y);
    const stanley::Result r = stanley::compute(s, current_global_path, closest_idx, p);

    //stash for debug viz
    last_closest_idx_       = closest_idx;
    last_cross_track_error_ = r.cte;
    last_heading_error_     = r.heading_error;
    last_heading_term_      = r.heading_term;
    last_cte_term_          = r.cte_term;
    last_steering_cmd_      = r.steering;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "heading_err=%.3f, cte=%.3f, steer=%.3f, speed=%.2f, idx=%zu",
        r.heading_error, r.cte, r.steering, r.speed, closest_idx);

    ackermann_msgs::msg::AckermannDrive drive;
    drive.steering_angle = r.steering;
    drive.speed = r.speed;

    ackermann_msgs::msg::AckermannDriveStamped stamp;
    stamp.drive = drive;
    stamp.header.frame_id = local_frame_id;
    stamp.header.stamp = this->now();

    return stamp;

}

size_t Stanley_Controller_Node::find_closest_point(double x, double y) {

    const size_t n = current_global_path.poses.size();
    if (n == 0) return 0;

    auto dist_to = [&](size_t i) {
        double dx = x - current_global_path.poses[i].pose.position.x;
        double dy = y - current_global_path.poses[i].pose.position.y;
        return std::sqrt(dx * dx + dy * dy);
    };

    auto full_scan = [&]() {
        size_t best_idx = 0;
        double best = std::numeric_limits<double>::max();
        for (size_t i = 0; i < n; i++) {
            double d = dist_to(i);
            if (d < best) { best = d; best_idx = i; }
        }
        return best_idx;
    };

    //first call (or after a new path arrives): full scan to locate ourselves
    if (!closest_idx_initialized_) {
        prev_closest_idx_ = full_scan();
        closest_idx_initialized_ = true;
        return prev_closest_idx_;
    }

    //subsequent calls: only search forward from the last known index.
  
    size_t best_idx  = prev_closest_idx_;
    double best_dist = std::numeric_limits<double>::max();

    for (size_t k = 0; k < closest_point_window_; k++) {
        size_t i = (prev_closest_idx_ + k) % n;   //wrap for closed loop
        double d = dist_to(i);
        if (d < best_dist) { best_dist = d; best_idx = i; }
    }

    //recovery: the window only searches forward, so after a pose jump re-search the whole path
    const rclcpp::Time t = this->now();
    if (best_dist > closest_point_recovery_dist_ &&
        (last_recovery_scan_.nanoseconds() == 0 || (t - last_recovery_scan_).seconds() >= 1.0)) {
        best_idx = full_scan();
        last_recovery_scan_ = t;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "closest point search lost track, re-acquiring (idx=%zu)", best_idx);
    }

    prev_closest_idx_ = best_idx;
    return best_idx;

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
    stamp.header.frame_id = local_frame_id;
    stamp.header.stamp = this->now();

    return stamp;

}

void Stanley_Controller_Node::init_parameters() {

    //declare parameters
    this->declare_parameter<std::string>("global_path_topic", "/global_planner/path");
    this->declare_parameter<std::string>("dead_man_active_topic", "/hal/dead_man_switch");
    this->declare_parameter<std::string>("ackermann_control_topic", "/hal/drive/autonomy");
    this->declare_parameter<std::string>("odom_topic", "/hal/localization/odom");
    this->declare_parameter<std::string>("speed_topic", "/hal/odom");
    

    this->declare_parameter<std::string>("global_frame_id", "map");
    this->declare_parameter<std::string>("local_frame_id", "base_link");

    //no defaults for values that differ between car and sim:
    //a missing or misspelled key stops the node instead of running on other gains
    this->declare_parameter<double>("speed_limit");
    this->declare_parameter<double>("k_e");
    this->declare_parameter<double>("k_h");
    this->declare_parameter<double>("control_rate_hz");

    this->declare_parameter<bool>("speed_limit_active", true);
    this->declare_parameter<double>("max_steering_angle", 0.52);
    this->declare_parameter<double>("k_soft", 0.5);
    this->declare_parameter<double>("wheelbase", 0.324);

    this->declare_parameter<bool>("enable_debug_vis", true);
    this->declare_parameter<double>("debug_vis_rate_hz", 20.0);

    this->declare_parameter<int>("closest_point_window", 40);
    this->declare_parameter<double>("closest_point_recovery_dist", 2.0);
    this->declare_parameter<double>("pose_timeout_s", 0.5);

    //init parameters
    global_path_topic       = this->get_parameter("global_path_topic").as_string();
    dead_man_active_topic   = this->get_parameter("dead_man_active_topic").as_string();
    ackermann_control_topic = this->get_parameter("ackermann_control_topic").as_string();
    odom_topic              = this->get_parameter("odom_topic").as_string();
    speed_topic             = this->get_parameter("speed_topic").as_string();
   

    global_frame_id = this->get_parameter("global_frame_id").as_string();
    local_frame_id  = this->get_parameter("local_frame_id").as_string();

    speed_limit_enable = this->get_parameter("speed_limit_active").as_bool();
    speed_limit        = this->get_parameter("speed_limit").as_double();
    max_steering_angle = this->get_parameter("max_steering_angle").as_double();
    k_e                = this->get_parameter("k_e").as_double();
    k_h                = this->get_parameter("k_h").as_double();
    k_soft             = this->get_parameter("k_soft").as_double();
    wheelbase          = this->get_parameter("wheelbase").as_double();

    enable_debug_vis  = this->get_parameter("enable_debug_vis").as_bool();
    control_rate_hz   = this->get_parameter("control_rate_hz").as_double();
    debug_vis_rate_hz = this->get_parameter("debug_vis_rate_hz").as_double();

    const int64_t window         = this->get_parameter("closest_point_window").as_int();
    closest_point_recovery_dist_ = this->get_parameter("closest_point_recovery_dist").as_double();
    pose_timeout_s               = this->get_parameter("pose_timeout_s").as_double();

    //validate: these divide, size a timer or bound a loop, so <= 0 would crash or hang the node
    auto require_positive = [this](const char* name, double value) {
        if (!(value > 0.0)) {
            RCLCPP_FATAL(this->get_logger(), "parameter %s must be > 0 (got %g)", name, value);
            throw std::invalid_argument(std::string("stanley_controller: ") + name + " must be > 0");
        }
    };
    require_positive("control_rate_hz", control_rate_hz);
    require_positive("debug_vis_rate_hz", debug_vis_rate_hz);   //turn debug off with enable_debug_vis
    require_positive("closest_point_window", static_cast<double>(window));
    require_positive("closest_point_recovery_dist", closest_point_recovery_dist_);
    require_positive("pose_timeout_s", pose_timeout_s);

    vis_every_n_ = static_cast<size_t>(std::max(1.0, std::round(control_rate_hz / debug_vis_rate_hz)));
    closest_point_window_ = static_cast<size_t>(window);

    //initialize state and internal variables
    dead_man_active.data = false;
    controller_state = stanley_state_::INACTIVE;
    current_velocity = 0.0;

}

void Stanley_Controller_Node::publish_debug_path() {

    //the path Stanley follows (after duplicate points are dropped), grey
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_id;
    m.header.stamp = this->now();
    m.ns = "path";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.05;
    m.color.r = 0.4; m.color.g = 0.4; m.color.b = 0.4; m.color.a = 0.8;
    for (const auto& p : current_global_path.poses) {
        geometry_msgs::msg::Point pt;
        pt.x = p.pose.position.x; pt.y = p.pose.position.y; pt.z = 0.02;
        m.points.push_back(pt);
    }

    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(m);
    debug_path_pub_->publish(arr);

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
    const double yaw = extractYaw(base_link_pose.orientation);

    const double fx = base_link_pose.position.x + wheelbase * std::cos(yaw);
    const double fy = base_link_pose.position.y + wheelbase * std::sin(yaw);

    double cx = 0, cy = 0, tx = 1, ty = 0;
    const auto& poses = current_global_path.poses;
    if (closest_idx < poses.size()) {
        cx = poses[closest_idx].pose.position.x;
        cy = poses[closest_idx].pose.position.y;
        //wrap the same way calculate_control does, so the tangent arrow
        //matches what the controller is actually using at loop closure
        size_t next = (closest_idx + 1) % poses.size();
        tx = poses[next].pose.position.x - cx;
        ty = poses[next].pose.position.y - cy;
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

    // 1. Front axle (yellow)
    {
        visualization_msgs::msg::Marker m;
        make_header(m, "front_axle", 0, visualization_msgs::msg::Marker::SPHERE);
        m.pose.position.x = fx; m.pose.position.y = fy; m.pose.position.z = 0.1;
        m.scale.x = m.scale.y = m.scale.z = 0.15;
        m.color.r = 1.0; m.color.g = 1.0; m.color.a = 1.0;
        arr.markers.push_back(m);
    }

    // 2. Closest path point (green)
    {
        visualization_msgs::msg::Marker m;
        make_header(m, "closest_point", 0, visualization_msgs::msg::Marker::SPHERE);
        m.pose.position.x = cx; m.pose.position.y = cy; m.pose.position.z = 0.1;
        m.scale.x = m.scale.y = m.scale.z = 0.18;
        m.color.g = 1.0; m.color.a = 1.0;
        arr.markers.push_back(m);
    }
    
    // 3. CTE line (colour by sign, intensity by magnitude)
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

    // 4. Path tangent arrow (green)
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

    // 5. Vehicle heading arrow (orange)
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

    // 6. Steering command arrow (magenta)
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

    // 7. Debug text above the car
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
