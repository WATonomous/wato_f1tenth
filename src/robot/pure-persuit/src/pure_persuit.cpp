#include "pure_persuit.hpp"

Pure_Persuit_Node::Pure_Persuit_Node () : Node ("pure_persuit_node") {

    //parameters
    Pure_Persuit_Node::init_parameters();

    //publisher
    controls_pub_ = this->create_publisher<ackermann_ds>(
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

    control_loop_timer = this->create_wall_timer(
        std::chrono::milliseconds(20),
        [this](){Pure_Persuit_Node::control_timer_callback();}
    );
}

//the guy that appies the control law
void Pure_Persuit_Node::control_timer_callback() {

    state_manage();

    //don't apply any control if the controler is not active, stop everything
    if (controller_state == state_::INACTIVE) {
        Pure_Persuit_Node::dead_stop();
        return;
    } 

    double v = 0;
    geometry_msgs::msg::Point p;

    if (controller_state == state_::GLOBAL_FOLLOW)
        Pure_Persuit_Node::get_global_waypoint_velocity(p,v);
    else if (controller_state == state_::LOCAL_FOLLOW)
        Pure_Persuit_Node::get_local_waypoint_velocity(p,v);

    //apply the control law
    ackermann_ds control_command = Pure_Persuit_Node::calculate_control(p,v);

    //publish control inputs
    controls_pub_->publish(control_command);

}

ackermann_ds Pure_Persuit_Node::calculate_control(
    const geometry_msgs::msg::Point &target_point, 
    const double &velocity) {

}
void Pure_Persuit_Node::state_manage () {

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

void Pure_Persuit_Node::init_parameters () {

    //declare parameters
    this->declare_parameter<std::string>("global_frame_id","map");
    this->declare_parameter<std::string>("local_frame_id","base_link");

    this->declare_parameter<std::string>("global_path_topic","");
    this->declare_parameter<std::string>("local_path_topic","/local_path");
    this->declare_parameter<std::string>("overtake_ready_topic","/overtake_ready");
    this->declare_parameter<std::string>("dead_man_active_topic","/dead_man_switch");
    this->declare_parameter<std::string>("ackermann_control_topic","/drive/autonomy");

    this->declare_parameter<bool>("overtake_enable",false);
    this->declare_parameter<double>("look_ahead_distance",0.3);

    this->declare_parameter<bool>("speed_limit_active", true);
    this->declare_parameter<double>("speed_limit", 0.25);

    //init parameters
    global_frame_id = this->get_parameter("global_frame_id").as_string();
    local_frame_id = this->get_parameter("local_frame_id").as_string();

    global_path_topic = this->get_parameter("global_path_topic").as_string();
    local_path_topic = this->get_parameter("local_path_topic").as_string();
    overtake_ready_topic = this->get_parameter("overtake_ready_topic").as_string();
    dead_man_active_topic = this->get_parameter("dead_man_active_topic").as_string();
    ackermann_control_topic = this->get_parameter("ackermann_control_topic").as_string();

    overtaking_enable = this->get_parameter("overtake_enable").as_bool();
    look_ahead_distance = this->get_parameter("look_ahead_distance").as_double();

    speed_limit_enable = this->get_parameter("speed_limit_active").as_bool();
    speed_limit = this->get_parameter("speed_limit").as_double();

    //initalize state and internal variables
    dead_man_active.data = false;
    overtake_active.data = false;

    controller_state = state_::INACTIVE;

    current_global_path = nav_msgs::msg::Path();
    current_local_path = nav_msgs::msg::Path();
}