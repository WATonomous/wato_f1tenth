#include "pure_persuit.hpp"

Pure_Persuit_Node::Pure_Persuit_Node () : Node ("pure_persuit_node") {

    //parameters
    Pure_Persuit_Node::init_parameters();

    //publisher
    controls_pub_ = this->create_publisher< ackermann_msgs::msg::AckermannDriveStamped>(
        ackermann_control_topic, 10);

    //subscriptions
    
    auto latched_qos = rclcpp::QoS(1).transient_local().reliable();

    dead_man_sub_ = this->create_subscription<std_msgs::msg::Bool>( dead_man_active_topic, latched_qos, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {dead_man_active = *msg;}
    );

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        global_path_topic, latched_qos,
        [this](const nav_msgs::msg::Path::SharedPtr msg){current_global_path = *msg;}
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg){current_pose = *msg;}
    );

    //making this an options branch now as we don't have the local planing stuff working
    if (overtaking_enable) {

        overtake_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            overtake_ready_topic, latched_qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { overtake_active = *msg; }
        );

        local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            local_path_topic, 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) { current_local_path = *msg; }
        );
    }

}
/*
void Pure_Persuit_Node::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    //save the current pose
    current_pose = *msg;

    //updated controller state
    Pure_Persuit_Node::update_controller_state();

    //don't apply any control if the controler is not active, stop everything
    if (controller_state == state_::INACTIVE) {
        controls_pub_->publish(Pure_Persuit_Node::dead_stop());
        return;
    } 

    double v = 0;
    geometry_msgs::msg::Point p;

    if (controller_state == state_::GLOBAL_FOLLOW)
        Pure_Persuit_Node::get_global_waypoint(p,v);
    else if (controller_state == state_::LOCAL_FOLLOW)
        Pure_Persuit_Node::get_local_waypoint(p,v);

    //apply the control law
    ackermann_msgs::msg::AckermannDriveStamped control_command = Pure_Persuit_Node::calculate_control(p,v);

    //publish control inputs
    controls_pub_->publish(control_command);

}
*/

/*
assumption for this one  : 
- the local planner always gives all the cordinates in base_link frame
- the start of the local planning array will always be 0,0 so you just need to traverse up 
  to find the look ahead distance point
*/
void Pure_Persuit_Node::get_local_waypoint(geometry_msgs::msg::Point &current_point, double &current_velocity) {

    geometry_msgs::msg::Point target_point;
    double target_velocity;

    for (const auto point : current_local_path.poses) {

        double distance = Pure_Persuit_Node::find_distance(current_pose.pose.pose, point.pose);

        if (distance >= look_ahead_distance) {
            target_point = point.pose.position;
            target_velocity = point.pose.position.z;
            break;
        }
    }
}

double Pure_Persuit_Node::find_distance(geometry_msgs::msg::Pose current_location, geometry_msgs::msg::Pose destination) {
    return std::sqrt(std::pow(destination.position.x - current_location.position.x, 2) + 
        std::pow(destination.position.y - current_location.position.y, 2));
}

double Pure_Persuit_Node::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (std::pow(quat.y,2) + std::pow(quat.z,2)));
}
/*
assumption for this one  : 
- the global planner always gives all the cordinates in map frame, thus requiring a cordinate
  requiring a cordinate conversion before being able to apply the control law to it
*/
void Pure_Persuit_Node::get_global_waypoint(geometry_msgs::msg::Point &current_point, double &current_velocity) {

}

//the guy that appies the control law
void Pure_Persuit_Node::control_timer_callback() {

    //updated controller state
    Pure_Persuit_Node::update_controller_state();

    //don't apply any control if the controler is not active, stop everything
    if (controller_state == state_::INACTIVE) {
        controls_pub_->publish(Pure_Persuit_Node::dead_stop());
        return;
    } 

    double v = 0;
    geometry_msgs::msg::Point p;

    if (controller_state == state_::GLOBAL_FOLLOW)
        Pure_Persuit_Node::get_global_waypoint(p,v);
    else if (controller_state == state_::LOCAL_FOLLOW)
        Pure_Persuit_Node::get_local_waypoint(p,v);

    //apply the control law
    ackermann_msgs::msg::AckermannDriveStamped control_command = Pure_Persuit_Node::calculate_control(p,v);

    //publish control inputs
    controls_pub_->publish(control_command);

}

ackermann_msgs::msg::AckermannDriveStamped Pure_Persuit_Node::calculate_control(
    const geometry_msgs::msg::Point &target_point, 
    const double &velocity) {

    //this formula derivation should be a in muhtasim note's co-op 1 notes and should be a picture on the github
    double steering_angle = std::atan2(wheel_base * 2 * target_point.y, std::pow(look_ahead_distance, 2));

    if (steering_angle > max_steering_angle) {
        steering_angle = max_steering_angle;
    }

    if (steering_angle < -max_steering_angle) {
        steering_angle = -max_steering_angle;
    }

    ackermann_msgs::msg::AckermannDrive drive;
    drive.steering_angle = steering_angle;

    if (speed_limit_enable) {
        drive.speed = speed_limit;
    } else { 
        drive.speed = velocity;
    }

    ackermann_msgs::msg::AckermannDriveStamped stamp;
    stamp.drive = drive;
    stamp.header.frame_id = local_frame_id;
    stamp.header.stamp = this->now();

    return stamp;

}

void Pure_Persuit_Node::update_controller_state () {

    if (dead_man_active.data && !current_global_path.poses.empty()) {
       controller_state = state_::GLOBAL_FOLLOW; 
    } else {
        controller_state = state_::INACTIVE;
        RCLCPP_WARN(this->get_logger(), "DeadMan switch is off OR global path is empty");
    }

    if (overtaking_enable) {
       if (current_local_path.poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "could not transition to overtake mode, local path empty");
            return;
       }

        if (overtake_active.data && controller_state == state_::GLOBAL_FOLLOW) {
            controller_state = state_::LOCAL_FOLLOW;
        } else if (!overtake_active.data && controller_state == state_::LOCAL_FOLLOW) {
            controller_state = state_::GLOBAL_FOLLOW;
        }
    }

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

void Pure_Persuit_Node::init_parameters () {

    //declare parameters
    this->declare_parameter<std::string>("global_frame_id","map");
    this->declare_parameter<std::string>("local_frame_id","base_link");

    this->declare_parameter<std::string>("global_path_topic","");
    this->declare_parameter<std::string>("local_path_topic","/local_path");
    this->declare_parameter<std::string>("overtake_ready_topic","/overtake_ready");
    this->declare_parameter<std::string>("dead_man_active_topic","/dead_man_switch");
    this->declare_parameter<std::string>("ackermann_control_topic","/drive/autonomy");
    this->declare_parameter<std::string>("odom_topic","/odom");

    this->declare_parameter<bool>("overtake_enable",false);
    this->declare_parameter<double>("look_ahead_distance",0.3);

    this->declare_parameter<bool>("speed_limit_active", true);
    this->declare_parameter<double>("speed_limit", 0.25);
    this->declare_parameter<double>("wheel_base",0.31);
    this->declare_parameter<double>("max_steering_angle",0.52);

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
    look_ahead_distance = this->get_parameter("look_ahead_distance").as_double();

    speed_limit_enable = this->get_parameter("speed_limit_active").as_bool();
    speed_limit = this->get_parameter("speed_limit").as_double();
    wheel_base = this->get_parameter("wheel_base").as_double();
    max_steering_angle = this->get_parameter("max_steering_angle").as_double();

    //initalize state and internal variables
    dead_man_active.data = false;
    overtake_active.data = false;

    controller_state = state_::INACTIVE;

    current_global_path = nav_msgs::msg::Path();
    current_local_path = nav_msgs::msg::Path();
}