#include "fake_odom.hpp"

FakeOdom::FakeOdom() : Node ("fake_odom") {

    //frames declaration
    this->declare_parameter<std::string>("child_frame","base_link");
    this->declare_parameter<std::string>("header_frame","odom");
    this->declare_parameter<double>("inital_speed", 0.0);

    child_frame_name = this->get_parameter("child_frame").as_string();
    header_frame_name = this->get_parameter("header_frame").as_string();
    current_speed.data = static_cast<float>(this->get_parameter("inital_speed").as_double());

    //publisher
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom",10);

    //subscriptions
    tf_sub = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 10, std::bind(&FakeOdom::tf_listener, this, std::placeholders::_1));

    speed_sub = this->create_subscription<std_msgs::msg::Float32>(
        "/autodrive/f1tenth_1/speed", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg){
            current_speed = *msg;
    });

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);


    RCLCPP_INFO(this->get_logger(), "init the constructor for Fake odom");
}

// listen to the messages
void FakeOdom::tf_listener(const tf2_msgs::msg::TFMessage::SharedPtr tf_msg) {

    for (const auto &t : tf_msg->transforms) {
        if (t.child_frame_id == "f1tenth_1" && t.header.frame_id == "world") {

            nav_msgs::msg::Odometry tf_odom;

            tf_odom.pose.pose.position.x = t.transform.translation.x;
            tf_odom.pose.pose.position.y = t.transform.translation.y;
            tf_odom.pose.pose.position.z = t.transform.translation.z;

            tf_odom.pose.pose.orientation.w = t.transform.rotation.w;
            tf_odom.pose.pose.orientation.x = t.transform.rotation.x;
            tf_odom.pose.pose.orientation.y = t.transform.rotation.y;
            tf_odom.pose.pose.orientation.z = t.transform.rotation.z;

            tf_odom.header.stamp = t.header.stamp;

            FakeOdom::publish_msgs(tf_odom);

            break;

        }
    }
}

//transmit the tf into a odom topic and create a odom -> base_link tf
void FakeOdom::publish_msgs(const nav_msgs::msg::Odometry &odom_msg) {
    //publishing odom
    nav_msgs::msg::Odometry current_odom; 

    current_odom.header.frame_id = header_frame_name;
    current_odom.child_frame_id = child_frame_name;
    current_odom.header.stamp = odom_msg.header.stamp;

    current_odom.pose.pose.position = odom_msg.pose.pose.position;
    current_odom.pose.pose.orientation = odom_msg.pose.pose.orientation;
    current_odom.twist.twist.linear.x = current_speed.data;

    odom_pub->publish(current_odom);

    //publishing tf 
    geometry_msgs::msg::TransformStamped t;

    t.child_frame_id = child_frame_name;
    t.header.frame_id = header_frame_name;
    t.header.stamp = odom_msg.header.stamp;

    t.transform.translation.x = odom_msg.pose.pose.position.x;
    t.transform.translation.y = odom_msg.pose.pose.position.y;
    t.transform.translation.z = odom_msg.pose.pose.position.z;

    t.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster->sendTransform(t);

}

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeOdom>());
  rclcpp::shutdown();
  return 0;

}