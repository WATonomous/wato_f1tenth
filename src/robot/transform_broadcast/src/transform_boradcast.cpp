#include "transform_boradcast.hpp"

Transform_Node::Transform_Node () : Node ("Transform_Broadcast_Node") {

    //create subscriptions
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odom",10,std::bind(&Transform_Node::boradCastTransform,this,std::placeholders::_1));

    //intalize the tf2 broadcaster
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

}

void Transform_Node::boradCastTransform(const nav_msgs::msg::Odometry::SharedPtr msg) {
    
    t.child_frame_id = msg->child_frame_id;
    t.header.frame_id = msg->header.frame_id;
    t.header.stamp = msg->header.stamp;

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;
    
    tf_broadcaster->sendTransform(t);

}

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Transform_Node>());
  rclcpp::shutdown();
  return 0;

}