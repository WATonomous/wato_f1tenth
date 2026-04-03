#include "pure_persuit.hpp"

Pure_Persuit_Node::Pure_Persuit_Node () : Node ("pure_persuit_node") {

    //parameters
    Pure_Persuit_Node::init_parameters();

    //publisher
    controls_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        ackermann_control_topic, 10);

    //subscriptions
    
    auto latched_qos = rclcpp::QoS(1).transient_local().reliable();

    dead_man_sub_ = this->create_subscription<std_msgs::msg::Bool>( dead_man_active_topic, latched_qos, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {dead_man_active = &msg;}
    );

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        global_path_topic, latched_qos,
        [this](const nav_msgs::msg::Path::SharedPtr msg){current_global_path = *msg;}
    );

    //making this an options branch now as we don't have the local planing stuff working
    if (overtaking_enable) {

        overtake_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            overtake_ready_topic, latched_qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { overtake_active = &msg; }
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

    dead_man_active = false;
    overtake_active = false;
}