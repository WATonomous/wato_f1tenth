#include "pure_persuit.hpp"

/*
test & developmnet plan :
    tasks done:
    - compile it and get rid of any compilation bugs (done)
    - just start the node by itself -> should say no dead man (done)
    - manually send true to dead man -> should say no global path
    - launch the node with the global path and other stuff then launch pure persuit -> do nothing , send true -> should start to follow path at 0.5 m/s (done)
    - need to add and test dynamic look ahead distance -> car shold ossilate less on straights, but track the corners well (done)

    tasks that need to be done:
    - need to add and test point and velocity interpolation between 2 points -> should result in less jittery movemnet verify using log output (Jordan)

    long term (1 month):
    - need to figure out how to test and verify local path switching (need to make this shit first though ahhhhhh)
    - optmize the global path and pure persuit for better lap times

    notes & caviats :
    - currently state transitions are allowd no mater what, but if the way points are empty then car is just halted 
      might consider changing that in the future if problamatic during local planner testing
*/

Pure_Persuit_Node::Pure_Persuit_Node () : Node ("pure_persuit_node") {

    //parameters
    Pure_Persuit_Node::init_parameters();

    //publisher
    controls_pub_ = this->create_publisher< ackermann_msgs::msg::AckermannDriveStamped>(
        ackermann_control_topic, 10);

    //subscriptions
    
    auto latched_qos = rclcpp::QoS(1).transient_local().reliable();

    dead_man_sub_ = this->create_subscription<std_msgs::msg::Bool>( dead_man_active_topic, latched_qos, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            dead_man_active = *msg;
        }
    );

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        global_path_topic, latched_qos,
        [this](const nav_msgs::msg::Path::SharedPtr msg){
            current_global_path = *msg;
        }
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg){
            current_pose = *msg;
        }
    );

    speed_sub_= this->create_subscription<nav_msgs::msg::Odometry>(
        speed_topic, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg){
            current_velocity = msg->twist.twist.linear.x;
        }
    );

    look_ahead_pub_ = this->create_publisher<std_msgs::msg::Float32>("/debug/lookahead_distance",10);

    lookahead_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/debug/lookahead_point", 10);

    //making this an options branch now as we don't have the local planing stuff working
    if (overtaking_enable) {

        overtake_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            overtake_ready_topic, latched_qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { 
                overtake_active = *msg; 
            }
        );

        local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            local_path_topic, 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                current_local_path = *msg;
            }
        );
    }

    control_loop_timer = this->create_wall_timer (
        std::chrono::duration<double>(1.0 / control_rate_hz),
        [this](){
            control_timer_callback();
        }
    );

    //tf2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

/*
key assumption : the z value of the point holdes the velocity and always will
*/
void Pure_Persuit_Node::control_timer_callback() {

    //updated controller state
    Pure_Persuit_Node::update_controller_state();
    Pure_Persuit_Node::update_lookahead_distance();

    //don't apply any control if the controler is not active, stop everything
    if (controller_state == state_::INACTIVE) {
        RCLCPP_WARN(this->get_logger(), "Dead Man switch is off");
        controls_pub_->publish(Pure_Persuit_Node::dead_stop());
        return;
    } 

    std::optional<geometry_msgs::msg::Point> p;

    if (controller_state == state_::GLOBAL_FOLLOW) {

        p = Pure_Persuit_Node::get_global_waypoint();

    } else if (controller_state == state_::LOCAL_FOLLOW) {

        p = Pure_Persuit_Node::get_local_waypoint();

        // An empty or exhausted local path means there is no local command to
        // execute. The common no-waypoint check below stops the car.
        if (!p.has_value()) {

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "local path yielded no lookahead point, stopping");

        }

    }

    if (!p.has_value()) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
            "no look ahead point returned, stopping car");
        controls_pub_->publish(Pure_Persuit_Node::dead_stop());
        return;
    }

    // publish debug lookahead point for foxglove visualization
    if (enable_debug_vis) {
        publish_debug_vis(p.value());
    }

    //apply the control law
    ackermann_msgs::msg::AckermannDriveStamped control_command = Pure_Persuit_Node::calculate_control(p.value());

    //publish control inputs
    controls_pub_->publish(control_command);

}

void Pure_Persuit_Node::update_controller_state () {

    if (dead_man_active.data || force_dead_man_active) {

       controller_state = state_::GLOBAL_FOLLOW;

    } else {

        controller_state = state_::INACTIVE;

    }

    if (overtaking_enable && overtake_active.data &&
        controller_state == state_::GLOBAL_FOLLOW) {

        controller_state = state_::LOCAL_FOLLOW;

    }

}

/*
map-frame path in, base_link lookahead out. global and local share this: closest
index vs live pose, lookahead on the polyline, then a TF read at this instant.

closed_loop is the only difference, and it is confined to find_lookahead: the
global path is a lap so the lookahead wraps past the end of the array, the local
path is an open horizon so running off it is a real answer, not a wrap. the
closest index is a full scan either way.
*/
std::optional<geometry_msgs::msg::Point> Pure_Persuit_Node::get_waypoint_from_path(
    const nav_msgs::msg::Path &path, bool closed_loop) {

    if (path.poses.empty()) {

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
            "no waypoints in %s while in %s",
            closed_loop ? "global path" : "local path",
            closed_loop ? "GLOBAL_FOLLOW" : "LOCAL_FOLLOW");
        return std::nullopt;

    }

    const size_t current_pose_index = Pure_Persuit_Node::find_closest_index(path);

    const std::optional<geometry_msgs::msg::Point> target_waypoint_global =
        Pure_Persuit_Node::find_lookahead(path, current_pose_index, closed_loop);

    if (!target_waypoint_global.has_value()) {

        if (closed_loop) {
            RCLCPP_WARN(this->get_logger(), "no target look ahead point found | find_lookahead()");
        }
        return std::nullopt;

    }

    const std::optional<geometry_msgs::msg::Point> converted_waypoint =
        Pure_Persuit_Node::convert_to_local_frame(target_waypoint_global.value());

    if (!converted_waypoint.has_value() && closed_loop) {

        RCLCPP_WARN(this->get_logger(), "no target look ahead point found | convert_to_local_frame()");

    }

    return converted_waypoint;

}

std::optional<geometry_msgs::msg::Point> Pure_Persuit_Node::get_global_waypoint() {

    return Pure_Persuit_Node::get_waypoint_from_path(current_global_path, true);

}

std::optional<geometry_msgs::msg::Point> Pure_Persuit_Node::get_local_waypoint() {

    return Pure_Persuit_Node::get_waypoint_from_path(current_local_path, false);

}

/*
full scan for the nearest pose, used by both paths. caller guarantees a non-empty
path.

the global path used to get an incremental cached scan instead. that cache was
only ever advanced from the GLOBAL_FOLLOW branch, so it froze for the duration of
an overtake, and the catch-up walk it relied on stopped at the first local
minimum of euclidean distance -- which a corner between the stale index and the
car produces immediately. the result was an index pinned metres behind the car
for the rest of the lap, and a lookahead point to match. the scan it was avoiding
is ~150 points at 50 Hz, so it was never buying anything: no cache, nothing to go
stale.
*/
size_t Pure_Persuit_Node::find_closest_index(const nav_msgs::msg::Path &path) {

    size_t closest_index = 0;
    double closest_distance = Pure_Persuit_Node::find_distance(current_pose.pose.pose, path.poses[0].pose);

    for (size_t i = 1; i < path.poses.size(); i++) {

        double current_distance = Pure_Persuit_Node::find_distance(current_pose.pose.pose, path.poses[i].pose);
        if (current_distance < closest_distance) {
            closest_distance = current_distance;
            closest_index = i;
        }

    }

    return closest_index;

}

/*
finding lookahead distance from current vehicle position
interpolates between the bracketing waypoints so the returned point lies
precisely on the lookahead circumference (prevents jitter)

everything is measured against the live current_pose in the global frame, so the
returned point is correct at the instant of the call no matter how old the path
is. that is the whole reason the local path is now handled here rather than being
walked from the origin of a pre-transformed base_link path.

closed_loop distinguishes the two callers: the global path is a lap and wraps, the
local path is an open 6 m horizon whose end is genuinely the end.
*/
std::optional<geometry_msgs::msg::Point> Pure_Persuit_Node::find_lookahead(
    const nav_msgs::msg::Path &path, size_t current_vehicle_index, bool closed_loop) {

    const double ref_x = current_pose.pose.pose.position.x;
    const double ref_y = current_pose.pose.pose.position.y;
    const size_t n = path.poses.size();

    // case 1 : from current point to end of vector
    for (size_t i = current_vehicle_index + 1 ; i < n; i ++) {

        double distance = Pure_Persuit_Node::find_distance(current_pose.pose.pose, path.poses[i].pose);

        if (distance >= look_ahead_distance) {

            const auto &prev_pt = path.poses[i - 1].pose.position;
            const auto &curr_pt = path.poses[i].pose.position;
            return Pure_Persuit_Node::interpolate_lookahead_point(prev_pt, curr_pt, ref_x, ref_y, look_ahead_distance);

        }

    }

    //case 2 : loop back from start to current as this represetns a closed loop
    if (closed_loop) {

        for (size_t i = 0; i < current_vehicle_index; i++) {

            double distance = Pure_Persuit_Node::find_distance(current_pose.pose.pose, path.poses[i].pose);

            if (distance >= look_ahead_distance) {

                // at the wrap boundary the previous waypoint is the last one in the path
                size_t prev_idx = (i == 0) ? (n - 1) : (i - 1);
                const auto &prev_pt = path.poses[prev_idx].pose.position;
                const auto &curr_pt = path.poses[i].pose.position;
                return Pure_Persuit_Node::interpolate_lookahead_point(prev_pt, curr_pt, ref_x, ref_y, look_ahead_distance);

            }

        }

    }

    /*
      case 3 :
      could not find suitble point, return nullopt, caller choses how to handle.
      on the global path this should in theory never happen. on the local path it
      just means the horizon ran out ahead of the lookahead circle, and the caller
      falls back to the global line.
    */
    if (closed_loop) {

        RCLCPP_ERROR(this->get_logger(), "could not find lookahead point");

    }

    return std::nullopt;
}

/*
function author : Jordan Khatri
interpolates between prev_pt (inside the lookahead circle) and curr_pt (outside it)
to synthesize a point that lies exactly on the lookahead circumference. Velocity (z)
is linearly interpolated along the segment using the same parameter t.

math : parametrize the segment P(t) = prev + t*(curr - prev) and solve
|P(t) - ref|^2 = lookahead^2. Pick the forward root in [0,1].
*/
geometry_msgs::msg::Point Pure_Persuit_Node::interpolate_lookahead_point(
    const geometry_msgs::msg::Point &prev_pt,
    const geometry_msgs::msg::Point &curr_pt,
    double ref_x, double ref_y,
    double lookahead) {

    double dx = curr_pt.x - prev_pt.x;
    double dy = curr_pt.y - prev_pt.y;
    double fx = prev_pt.x - ref_x;
    double fy = prev_pt.y - ref_y;

    double a = dx * dx + dy * dy;
    double b = 2.0 * (fx * dx + fy * dy);
    double c = fx * fx + fy * fy - lookahead * lookahead;

    double discriminant = b * b - 4.0 * a * c;

    // degenerate segment or no real intersection: bail out to curr_pt
    if (a < 1e-9 || discriminant < 0.0) {
        return curr_pt;
    }

    double sqrt_disc = std::sqrt(discriminant);
    double t1 = (-b - sqrt_disc) / (2.0 * a);
    double t2 = (-b + sqrt_disc) / (2.0 * a);

    double t;
    if (t2 >= 0.0 && t2 <= 1.0) {
        t = t2;  // forward root
    } else if (t1 >= 0.0 && t1 <= 1.0) {
        t = t1;
    } else {
        return curr_pt;
    }

   

    geometry_msgs::msg::Point result;
    result.x = prev_pt.x + t * dx;
    result.y = prev_pt.y + t * dy;
    result.z = prev_pt.z + t * (curr_pt.z - prev_pt.z);

    return result;
}

std::optional<geometry_msgs::msg::Point> Pure_Persuit_Node::convert_to_local_frame(
    const geometry_msgs::msg::Point &global_point) {

    geometry_msgs::msg::TransformStamped t;

    try {

        t = tf_buffer_->lookupTransform(local_frame_id, global_frame_id, tf2::TimePointZero);

    } catch (const tf2::TransformException & ex) {

        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            local_frame_id.c_str(), global_frame_id.c_str(), ex.what());
        return std::nullopt;

    }

    return Pure_Persuit_Node::transfrom_point_(global_point,t.transform);

}

geometry_msgs::msg::Point Pure_Persuit_Node::transfrom_point_(

    const geometry_msgs::msg::Point &point_, 
    const geometry_msgs::msg::Transform &t_) {
 
    geometry_msgs::msg::Point p;
    
    double theta = Pure_Persuit_Node::extractYaw(t_.rotation);

    p.x = (std::cos(theta) * point_.x - std::sin(theta)* point_.y) + t_.translation.x;
    p.y = (std::sin(theta) * point_.x + std::cos(theta)* point_.y) + t_.translation.y;
    p.z = point_.z; // the speed

    return p;
    
}

ackermann_msgs::msg::AckermannDriveStamped Pure_Persuit_Node::calculate_control(
    const geometry_msgs::msg::Point &target_point) {

    double steering_angle = kp_gain * (2 * target_point.y / std::pow(look_ahead_distance, 2));

    if (steering_angle > max_steering_angle) {
        steering_angle = max_steering_angle;
    }

    if (steering_angle < -max_steering_angle) {
        steering_angle = -max_steering_angle;
    }

    ackermann_msgs::msg::AckermannDrive drive;
    drive.steering_angle = steering_angle;

    drive.speed = target_point.z;

    if (speed_limit_enable && drive.speed > speed_limit) {
        drive.speed = speed_limit;
    }

    ackermann_msgs::msg::AckermannDriveStamped stamp;
    stamp.drive = drive;
    stamp.header.frame_id = local_frame_id;
    stamp.header.stamp = this->now();

    return stamp;

}

ackermann_msgs::msg::AckermannDriveStamped Pure_Persuit_Node::dead_stop() {

    ackermann_msgs::msg::AckermannDrive drive;
    drive.speed = 0.0;
    drive.steering_angle = 0.0;

    ackermann_msgs::msg::AckermannDriveStamped stamp;
    stamp.drive = drive;
    stamp.header.frame_id = local_frame_id;
    stamp.header.stamp = this->now();

    return stamp;

}

double Pure_Persuit_Node::find_distance(geometry_msgs::msg::Pose current_location, geometry_msgs::msg::Pose destination) {

    return std::sqrt(std::pow(destination.position.x - current_location.position.x, 2) + 
        std::pow(destination.position.y - current_location.position.y, 2));

}

double Pure_Persuit_Node::extractYaw(const geometry_msgs::msg::Quaternion &quat) {

    return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (std::pow(quat.y,2) + std::pow(quat.z,2)));

}

void Pure_Persuit_Node::update_lookahead_distance() {

    look_ahead_distance = std::clamp(max_lookahead * current_velocity / lookahead_ratio, min_lookahead, max_lookahead);
}

void Pure_Persuit_Node::publish_debug_vis(geometry_msgs::msg::Point look_ahead_point_p) {

    //for debug purpose
    std_msgs::msg::Float32 ld;
    ld.data = look_ahead_distance;
    look_ahead_pub_->publish(ld);

    // publish debug lookahead point for foxglove / rviz visualization
    const auto stamp = this->now();

    visualization_msgs::msg::Marker lookahead_point;
    lookahead_point.header.stamp = stamp;
    lookahead_point.header.frame_id = local_frame_id;
    lookahead_point.ns = "lookahead";
    lookahead_point.id = 1;
    lookahead_point.type = visualization_msgs::msg::Marker::CYLINDER;
    lookahead_point.action = visualization_msgs::msg::Marker::ADD;
    lookahead_point.pose.position = look_ahead_point_p;
    lookahead_point.pose.orientation.w = 1.0;
    lookahead_point.scale.x = 1.0 / 3.0;
    lookahead_point.scale.y = 1.0 / 3.0;
    lookahead_point.scale.z = 0.08 / 3.0;
    lookahead_point.color.r = 1.0;
    lookahead_point.color.g = 0.1;
    lookahead_point.color.b = 0.1;
    lookahead_point.color.a = 0.95;
    lookahead_point_pub_->publish(lookahead_point);

}

void Pure_Persuit_Node::init_parameters () {

    //declare parameters
    this->declare_parameter<std::string>("global_frame_id","map");
    this->declare_parameter<std::string>("local_frame_id","base_link");

    this->declare_parameter<std::string>("global_path_topic","/global_planner/path");
    // map-frame local path is the controller contract; /local_path is leftover
    this->declare_parameter<std::string>("local_path_topic","/local_path_map");
    this->declare_parameter<std::string>("overtake_ready_topic","/overtake_ready");
    this->declare_parameter<std::string>("dead_man_active_topic","/dead_man_switch");
    this->declare_parameter<std::string>("ackermann_control_topic","/drive/autonomy");
    this->declare_parameter<std::string>("odom_topic","/odom");

    this->declare_parameter<bool>("overtake_enable",false);
    this->declare_parameter<bool>("force_dead_man_active",false);
    this->declare_parameter<double>("control_rate_hz", 50.0);

    this->declare_parameter<bool>("speed_limit_active", true);
    this->declare_parameter<double>("speed_limit", 10.0);

    this->declare_parameter<double>("max_steering_angle",0.52);

    this->declare_parameter<double>("kp_gain", 0.15);

    this->declare_parameter<double>("max_lookahead",2.0);
    this->declare_parameter<double>("min_lookahead",1.0);
    this->declare_parameter<double>("lookahead_ratio",6.0);

    //this->declare_parameter<std::string>("speed_topic","/ekf/odom");
    this->declare_parameter<std::string>("speed_topic","/autodrive/roboracer_1/odom");

    //DEBUG VIS
    this->declare_parameter<bool>("enable_debug_vis",true);

    //init parameters
    global_frame_id = this->get_parameter("global_frame_id").as_string();
    local_frame_id = this->get_parameter("local_frame_id").as_string();

    global_path_topic = this->get_parameter("global_path_topic").as_string();
    local_path_topic = this->get_parameter("local_path_topic").as_string();
    overtake_ready_topic = this->get_parameter("overtake_ready_topic").as_string();
    dead_man_active_topic = this->get_parameter("dead_man_active_topic").as_string();
    ackermann_control_topic = this->get_parameter("ackermann_control_topic").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();

    overtaking_enable = this->get_parameter("overtake_enable").as_bool();
    force_dead_man_active = this->get_parameter("force_dead_man_active").as_bool();
    control_rate_hz = std::max(0.1, this->get_parameter("control_rate_hz").as_double());

    speed_limit_enable = this->get_parameter("speed_limit_active").as_bool();
    speed_limit = this->get_parameter("speed_limit").as_double();
    max_steering_angle = this->get_parameter("max_steering_angle").as_double();

    kp_gain = this->get_parameter("kp_gain").as_double();

    max_lookahead = this->get_parameter("max_lookahead").as_double();
    min_lookahead = this->get_parameter("min_lookahead").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();

    speed_topic = this->get_parameter("speed_topic").as_string();

    enable_debug_vis = this->get_parameter("enable_debug_vis").as_bool();

    //initalize state and internal variables
    dead_man_active.data = false;
    overtake_active.data = false;

    controller_state = state_::INACTIVE;
    look_ahead_distance = 0.5;

}

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pure_Persuit_Node>());
  rclcpp::shutdown();
  return 0;

}
