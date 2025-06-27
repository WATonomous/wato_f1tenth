#include "odom_v2.hpp"

ODOM::ODOM () : Node ("odom") {

    //parameters

    this->declare_parameter<std::string>("right_encoder_topic","/autodrive/f1tenth_1/right_encoder");
    this->declare_parameter<std::string>("left_encoder_topic","/autodrive/f1tenth_1/left_encoder");
    this->declare_parameter<std::string>("steering_topic","/autodrive/f1tenth_1/steering");
    this->declare_parameter<std::string>("odom_topic","/odom");
    this->declare_parameter<double>("wheel_radius",0.0590 );
    this->declare_parameter<double>("wheel_base",0.3240);

    right_topic = this->get_parameter("right_encoder_topic").as_string();
    left_topic = this->get_parameter("left_encoder_topic").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    steering_topic = this->get_parameter("steering_topic").as_string();
    wheel_base = this->get_parameter("wheel_base").as_double();
    wheel_radius = this->get_parameter("wheel_radius").as_double();

    // subs
    //ensure that both left and right encoder have the same time stamp

    left_sub.subscribe(this,left_topic);
    right_sub.subscribe(this,right_topic);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), left_sub, right_sub);
    sync_->registerCallback(std::bind(&ODOM::odom_callback,this,std::placeholders::_1,std::placeholders::_2));

    steering_sub = this->create_subscription<std_msgs::msg::Float32>(
        steering_topic,10,std::bind(&ODOM::steering_callback,this,std::placeholders::_1));
    
    //pubs

    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic,10);

}

void ODOM::steering_callback(std_msgs::msg::Float32::SharedPtr msg) {
    steering_data = *msg;
}

