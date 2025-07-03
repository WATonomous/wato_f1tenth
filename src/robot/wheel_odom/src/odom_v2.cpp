#include "odom_v2.hpp"

ODOM::ODOM () : Node ("odom") {

    //parameters

    this->declare_parameter<std::string>("right_encoder_topic","/autodrive/f1tenth_1/right_encoder");
    this->declare_parameter<std::string>("left_encoder_topic","/autodrive/f1tenth_1/left_encoder");
    this->declare_parameter<std::string>("steering_topic","/autodrive/f1tenth_1/steering");
    this->declare_parameter<std::string>("odom_topic","/odom");
    this->declare_parameter<std::string>("header_frame","odom");
    this->declare_parameter<std::string>("child_frame","base_link");
    this->declare_parameter<double>("wheel_radius",0.0590 );
    this->declare_parameter<double>("wheel_base",0.3240);
    this->declare_parameter<double>("inital_x",0.7412);
    this->declare_parameter<double>("inital_y",3.1583);
    this->declare_parameter<double>("inital_theta",-M_PI/2);
    this->declare_parameter<double>("inital_velocity",0.0);
    this->declare_parameter<double>("inital_angulr_velocity",0.0);

    right_topic = this->get_parameter("right_encoder_topic").as_string();
    left_topic = this->get_parameter("left_encoder_topic").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    steering_topic = this->get_parameter("steering_topic").as_string();
    wheel_base = this->get_parameter("wheel_base").as_double();
    wheel_radius = this->get_parameter("wheel_radius").as_double();
    x = this->get_parameter("inital_x").as_double();
    y = this->get_parameter("inital_y").as_double();
    theta = this->get_parameter("inital_theta").as_double();
    velocity = this->get_parameter("inital_velocity").as_double();
    child_frame = this->get_parameter("child_frame").as_string();
    header_frame = this->get_parameter("header_frame").as_string();
    angular_velocity = this->get_parameter("inital_angulr_velocity").as_double();

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


/**
 * @brief
 * the function uses the most recent data from the wheel encoders
 * to update the robots pose : x,y,yaw,linear x velocity and 
 * angular velocity. the update scheme is carried out using a 'tricycle' 
 * model. 
 * 
 * @return
 * the robot's updated pose 
 */
void ODOM::odom_callback(const sensor_msgs::msg::JointState::ConstSharedPtr &left,
                         const sensor_msgs::msg::JointState::ConstSharedPtr &right) {

    // initalize the encoders first state

    if (!intalize_prev) {

        if (left && right) {
            
            left_prev = *left;
            right_prev = *right;

            RCLCPP_INFO(this->get_logger(), "initalized encoder");
            intalize_prev = true;

        }

        return;
    }

   //get steering angle and calcuate avg_velocity from encoders

   double phi = steering_data.data;
   double dt = (left->header.stamp.sec + left->header.stamp.nanosec * 1.0e-9) 
                -(left_prev.header.stamp.sec + left_prev.header.stamp.nanosec * 1.0e-9); 

   double left_delta = left->position.at(0) - left_prev.position.at(0);   
   double right_delta = right->position.at(0) - right_prev.position.at(0);   

   double left_distance = left_delta * wheel_radius;
   double right_distance = right_delta * wheel_radius;

   double left_velocity = left_distance / dt;
   double right_velocity = right_distance / dt;

   double avg_velocity = (right_velocity + left_velocity) / 2.0;

   //store the previous values

   right_prev = *right;
   left_prev = *left;

   //calculate the the new positon

   x = x + avg_velocity * std::cos(theta) * dt;
   y = y + avg_velocity * std::sin(theta) * dt;
   theta = theta + (avg_velocity/wheel_base) * std::tan(phi) * dt;
   velocity = avg_velocity;
   angular_velocity = avg_velocity / wheel_base;

   //publish the odom

   ODOM::publish_odom();

}

/**
 * @brief
 * takes the updated state and publishes  it  
 * 
 * @return
 * publishes a odometry message 
 */
void ODOM::publish_odom () {

    nav_msgs::msg::Odometry odom_now;

    tf2::Quaternion q;
    q.setRPY(0,0,theta);

    odom_now.child_frame_id = child_frame;
    odom_now.header.frame_id = header_frame;
    odom_now.header.stamp = left_prev.header.stamp;

    odom_now.pose.pose.position.y = y;
    odom_now.pose.pose.position.x = x;

    odom_now.pose.pose.orientation.x = q.getX();
    odom_now.pose.pose.orientation.y = q.getY();
    odom_now.pose.pose.orientation.z = q.getZ();
    odom_now.pose.pose.orientation.w = q.getW();

    odom_now.twist.twist.linear.x = velocity;
    odom_now.twist.twist.angular.z = angular_velocity;

    odom_pub->publish(odom_now);

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ODOM>());
  rclcpp::shutdown();
  return 0;
}