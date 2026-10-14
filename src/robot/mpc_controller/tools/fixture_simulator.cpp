#include "mpc_controller/scene.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <global_planner/msg/reference_track.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class FixtureSimulator : public rclcpp::Node {
public:
  FixtureSimulator() : Node("mpc_fixture_simulator")
  {
    const auto scene=declare_parameter<std::string>("scene","solo");
    declare_parameter("publish_state",true);declare_parameter("publish_grid",true);
    if (scene!="solo" && scene!="obstacle" && scene!="blocked") {
      throw std::invalid_argument("Unknown fixture scene");
    }
    auto latched=rclcpp::QoS(1).reliable().transient_local();
    reference_pub_=create_publisher<global_planner::msg::ReferenceTrack>("/global_planner/reference_track",latched);
    grid_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap",1);
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/odom",1);
    clearance_pub_=create_publisher<std_msgs::msg::Float64>("/mpc/ground_truth_clearance",1);
    selected_sub_=create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd",1,
      [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        mpc::Command u{msg->drive.speed,msg->drive.steering_angle};
        command_=mpc::validCommand(u) ? u : mpc::Command{};command_at_=mpc::Clock::now();
      });
    global_planner::msg::ReferenceTrack track;
    track.header.frame_id="map";track.header.stamp=now();track.path.header=track.header;
    for (int i=0;i<160;++i) {
      const double a=-std::acos(-1.)/2+i*2*std::acos(-1.)/160;
      geometry_msgs::msg::PoseStamped p;p.header=track.header;
      p.pose.position.x=10*std::cos(a);p.pose.position.y=10+10*std::sin(a);p.pose.position.z=1;
      p.pose.orientation.w=1;track.path.poses.push_back(p);
      global_planner::msg::TrackWidth width;width.left_m=3;width.right_m=3;
      track.widths.push_back(width);
    }
    reference_pub_->publish(track);
    grid_.header.frame_id="map";grid_.info.resolution=.1;
    grid_.info.width=260;grid_.info.height=260;
    grid_.info.origin.position.x=-13;grid_.info.origin.position.y=-3;grid_.info.origin.orientation.w=1;
    grid_.data.assign(260*260,0);
    if (scene!="solo") {
      for (int y=0;y<260;++y) {
        for (int x=0;x<260;++x) {
          const double mx=-13+(x+.5)*.1,my=-3+(y+.5)*.1;
          if (mx>2.8 && mx<3.2 && (scene=="blocked" || (my>-.3 && my<.9))) {
            grid_.data[y*260+x]=100;
          }
        }
      }
    }
    scene_=scene;
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this] {tick();});
  }
private:
  void tick()
  {
    if (mpc::Clock::now()-command_at_>std::chrono::milliseconds(60)) {command_={};}
    pose_=mpc::step(pose_,command_,.02);
    const auto stamp=now();
    if (get_parameter("publish_state").as_bool()) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp=stamp;odom.header.frame_id="map";odom.child_frame_id="base_link";
      odom.pose.pose.position.x=pose_.x;odom.pose.pose.position.y=pose_.y;
      odom.pose.pose.orientation.z=std::sin(pose_.yaw/2);odom.pose.pose.orientation.w=std::cos(pose_.yaw/2);
      odom.twist.twist.linear.x=command_.speed;odom_pub_->publish(odom);
    }
    if (ticks_++%5==0 && get_parameter("publish_grid").as_bool()) {
      grid_.header.stamp=stamp;grid_pub_->publish(grid_);
    }
    double available=3-std::abs(std::hypot(pose_.x,pose_.y-10)-10)-mpc::coveringRadius();
    if (scene_!="solo") {
      const double dx=std::max({2.8-pose_.x,0.,pose_.x-3.2});
      const double dy=scene_=="blocked" ? 0 : std::max({-.3-pose_.y,0.,pose_.y-.9});
      available=std::min(available,std::hypot(dx,dy)-mpc::coveringRadius());
    }
    std_msgs::msg::Float64 clearance;clearance.data=available;clearance_pub_->publish(clearance);
  }
  mpc::Pose pose_;
  mpc::Command command_;
  std::string scene_;
  int ticks_=0;
  mpc::Clock::time_point command_at_;
  nav_msgs::msg::OccupancyGrid grid_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<global_planner::msg::ReferenceTrack>::SharedPtr reference_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr clearance_pub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr selected_sub_;
};

int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<FixtureSimulator>());
  rclcpp::shutdown();return 0;
}
