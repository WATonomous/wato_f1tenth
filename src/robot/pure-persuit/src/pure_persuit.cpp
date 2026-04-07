#include "pure_persuit.hpp"

/*
currently state transitions are allowd no mater what, 
but if the way points are empty then car is just halted
might consider changing that in the future
*/

/*
test & developmnet plan :
    short term (next 3 days):
    - compile it and get rid of any compilation bugs
    - just start the node by itself -> should say no dead man
    - manually send true to dead man -> should say no global path
    - launch the node with the global path and other stuff then launch pure persuit -> do nothing , send true -> should start to follow path at 0.5 m/s
    - test our my muta's formulations for steering angle (direct) vs steven's (kp gain method)

    medium term (1 week);
    - need to add and test dynamic look ahead distance -> car shold ossilate less on straights, but track the corners well
    - need to add and test point and velocity interpolation between 2 points -> should result in less jittery movemnet verify using log output

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

    control_loop_timer = this->create_wall_timer (
        std::chrono::milliseconds(50), 
        [this](){control_timer_callback();}
    );

    //tf2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

/*
key assumption : the z value of the point holdes the velocity and always will
todo : come back and add dynamic look ahead distance, add adjustmnet for that
*/
void Pure_Persuit_Node::control_timer_callback() {

    //updated controller state
    Pure_Persuit_Node::update_controller_state();

    //don't apply any control if the controler is not active, stop everything
    if (controller_state == state_::INACTIVE) {
        RCLCPP_WARN(this->get_logger(), "Dead Man switch is off");
        controls_pub_->publish(Pure_Persuit_Node::dead_stop());
        return;
    } 

    std::optional<geometry_msgs::msg::Point> p;

    if (controller_state == state_::GLOBAL_FOLLOW)
        p = Pure_Persuit_Node::get_global_waypoint();
    else if (controller_state == state_::LOCAL_FOLLOW)
        p = Pure_Persuit_Node::get_local_waypoint();

    if (!p.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "no look ahead point returned, stopping car");
        controls_pub_->publish(Pure_Persuit_Node::dead_stop());
        return;
    }

    //apply the control law
    ackermann_msgs::msg::AckermannDriveStamped control_command = Pure_Persuit_Node::calculate_control(p.value());

    //publish control inputs
    controls_pub_->publish(control_command);

}

void Pure_Persuit_Node::update_controller_state () {

    if (dead_man_active.data) {

       controller_state = state_::GLOBAL_FOLLOW; 

    } else {

        controller_state = state_::INACTIVE;

    }

    if (overtaking_enable) {

        if (overtake_active.data && controller_state == state_::GLOBAL_FOLLOW) {

            controller_state = state_::LOCAL_FOLLOW;

        } else if (!overtake_active.data && controller_state == state_::LOCAL_FOLLOW) {

            controller_state = state_::GLOBAL_FOLLOW;

        }

    }

}

/*
assumption for this one  : 
- the global planner always gives all the cordinates in map frame, thus requiring a cordinate
  requiring a cordinate conversion before being able to apply the control law to it
*/
std::optional<geometry_msgs::msg::Point> Pure_Persuit_Node::get_global_waypoint() {

    //check the global path
    if (current_global_path.poses.empty()) {

        RCLCPP_WARN(this->get_logger(), "no waypoints in global path while in GLOBAL_FOLLOW state");
        return std::nullopt;

    }

    //find the current index corosponding to current location of vehicle
    size_t current_pose_index = Pure_Persuit_Node::find_current_position_index();

    //find the look_ahead point in the global frame
    std::optional<geometry_msgs::msg::Point> target_waypoint_global = Pure_Persuit_Node::find_lookahead_global(current_pose_index);

    if (!target_waypoint_global.has_value()) {

        RCLCPP_WARN(this->get_logger(), "no target look ahead point found | find_lookahead_global()");
        return std::nullopt;

    }

    //convert the point to the local frame
    std::optional<geometry_msgs::msg::Point> converted_waypoint = Pure_Persuit_Node::convert_to_local_frame(target_waypoint_global.value());

    if (!converted_waypoint.has_value()) {

        RCLCPP_WARN(this->get_logger(), "no target look ahead point found | convert_to_local_frame()");
        return std::nullopt;

    }

    return converted_waypoint;

}

size_t Pure_Persuit_Node::find_current_position_index() {

    static size_t prev_index_cache = 0;
    bool found_local_minimum = false;

    //use the prev_index_cache to find current distance prev_distance from point
    double prev_distance = Pure_Persuit_Node::find_distance(current_pose.pose.pose, current_global_path.poses[prev_index_cache].pose);
    size_t prev_index = prev_index_cache;

    for (size_t i = prev_index_cache + 1; i < current_global_path.poses.size(); i++) {

        double current_distance = Pure_Persuit_Node::find_distance(current_pose.pose.pose, current_global_path.poses[i].pose);
        if (current_distance <= prev_distance) {

            prev_index = i;
            prev_distance = current_distance;

        }

        if (current_distance > prev_distance) {

            found_local_minimum = true;
            break;

        }

    }

    if (!found_local_minimum) {

        for (size_t i = 0; i < prev_index_cache; i++) {

            double current_distance = Pure_Persuit_Node::find_distance(current_pose.pose.pose, current_global_path.poses[i].pose);
            if (current_distance <= prev_distance) {

                prev_index = i;
                prev_distance = current_distance;

            }

            if (current_distance > prev_distance) {
                
                found_local_minimum = true;
                break;

            }

        }

    }

    if (!found_local_minimum) {

        RCLCPP_INFO(
            this->get_logger(),
            "could not find the index of local minimum distance, returning last closest point"
        );

    }

    prev_index_cache = prev_index;
    return prev_index_cache;
    
}

// finding lookahead distance from current vehicle position
std::optional<geometry_msgs::msg::Point> Pure_Persuit_Node::find_lookahead_global(size_t current_vehicle_index) {

    // case 1 : from current point to end of vector
    for (size_t i = current_vehicle_index + 1 ; i < current_global_path.poses.size(); i ++) {

        double distance = Pure_Persuit_Node::find_distance(current_pose.pose.pose, current_global_path.poses[i].pose);

        if (distance >= look_ahead_distance) {

            return current_global_path.poses[i].pose.position;

        }

    }

    //case 2 : loop back from start to current as this represetns a closed loop
    for (size_t i = 0; i < current_vehicle_index; i++) {

        double distance = Pure_Persuit_Node::find_distance(current_pose.pose.pose, current_global_path.poses[i].pose);

        if (distance >= look_ahead_distance) {

            return current_global_path.poses[i].pose.position;

        }

    }

    /*
      case 3 : 
      could not find suitble point, return nullopt, caller choses how to handle
      this should in theory never happen
    */
    RCLCPP_ERROR(this->get_logger(), "could not find lookahead point");
    return std::nullopt;
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

/*
assumption for this one  : 
- the local planner always gives all the cordinates in base_link frame
- the start of the local planning array will always be 0,0 so you just need to traverse up 
  to find the look ahead distance point
- the z value of the point encodes the velocity at the desired point
*/
std::optional<geometry_msgs::msg::Point> Pure_Persuit_Node::get_local_waypoint() {

    if (current_local_path.poses.empty()) {

        RCLCPP_WARN(this->get_logger(), "no waypoints in local path while in LOCAL_FOLLOW state");
        return std::nullopt;

    }
    
    // i know it inits to zero, but just to be safe
    geometry_msgs::msg::Pose origin; 

    for (const auto &point : current_local_path.poses) {

        double distance = Pure_Persuit_Node::find_distance(origin, point.pose);

        if (distance >= look_ahead_distance) {

            return point.pose.position;

        }
    
    }

    // Fallback: path shorter than lookahead — use last point
    return current_local_path.poses.back().pose.position;

}

ackermann_msgs::msg::AckermannDriveStamped Pure_Persuit_Node::calculate_control(
    const geometry_msgs::msg::Point &target_point) {

    //this formula derivation should be a in muhtasim note's co-op 1 notes and should be a picture on the github
    double steering_angle = std::atan2(wheel_base * 2 * target_point.y, std::pow(look_ahead_distance, 2));

    //steven gong's method
    //double kp = 0.25; // this should be a ros parameter
    //double steering_angle = kp * (2 * y / std::pow(look_ahead_distance, 2));

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
        drive.speed = target_point.z;
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

void Pure_Persuit_Node::init_parameters () {

    //declare parameters
    this->declare_parameter<std::string>("global_frame_id","map");
    this->declare_parameter<std::string>("local_frame_id","base_link");

    this->declare_parameter<std::string>("global_path_topic","/global_planner/path");
    this->declare_parameter<std::string>("local_path_topic","/local_path");
    this->declare_parameter<std::string>("overtake_ready_topic","/overtake_ready");
    this->declare_parameter<std::string>("dead_man_active_topic","/dead_man_switch");
    this->declare_parameter<std::string>("ackermann_control_topic","/drive/autonomy");
    this->declare_parameter<std::string>("odom_topic","/odom");

    this->declare_parameter<bool>("overtake_enable",false);

    this->declare_parameter<double>("look_ahead_distance",0.15);
    this->declare_parameter<bool>("speed_limit_active", true);
    this->declare_parameter<double>("speed_limit", 0.5);

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
}

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pure_Persuit_Node>());
  rclcpp::shutdown();
  return 0;

}