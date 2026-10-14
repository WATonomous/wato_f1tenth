#include "mpc_controller/controller.hpp"
#include "mpc_controller/visualization.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <global_planner/msg/reference_track.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace mpc {
class Node : public rclcpp::Node {
public:
  Node() : rclcpp::Node("mpc_controller"), tf_(get_clock()), listener_(tf_)
  {
    if (declare_parameter<std::string>("model_profile",model_profile)!=model_profile) {
      throw std::invalid_argument("Generated model profile does not match launch; rebuild with MPC_CONFIG");
    }
    vehicle_frame_=declare_parameter<std::string>("vehicle_frame","base_link");
    max_tilt_=declare_parameter("max_planar_tilt_rad",.01);
    state_age_=declare_parameter("state_max_age_s",.06);
    grid_age_=declare_parameter("grid_max_age_s",.20);
    rear_offset_=declare_parameter("rear_axle_offset_x_m",0.);
    stall_ms_=declare_parameter("test_solver_stall_ms",0);
    visuals_enabled_=declare_parameter("publish_visualization",true);
    if (vehicle_frame_.empty() || !std::isfinite(max_tilt_) || max_tilt_<=0 || max_tilt_>.04 ||
      !std::isfinite(state_age_) || state_age_<=0 || state_age_>.25 ||
      !std::isfinite(grid_age_) || grid_age_<=0 || grid_age_>1 || !std::isfinite(rear_offset_) ||
      std::abs(rear_offset_)>1 || stall_ms_<0 || stall_ms_>1000) {
      throw std::invalid_argument("Invalid runtime configuration");
    }
    command_pub_=create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive/autonomy",1);
    path_pub_=create_publisher<nav_msgs::msg::Path>("/mpc/predicted_path",rclcpp::QoS(1).best_effort());
    diagnostics_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("/mpc/metrics",1);
    reason_pub_=create_publisher<std_msgs::msg::String>("/mpc/status",1);
    if (visuals_enabled_) {
      track_pub_=create_publisher<visualization::Markers>("/mpc/visualization/track",
        rclcpp::QoS(1).reliable().transient_local());
      visuals_pub_=create_publisher<visualization::Markers>("/mpc/visualization/cycle",rclcpp::QoS(1).best_effort());
      search_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("/mpc/visualization/search_space",rclcpp::QoS(1).best_effort());
      visuals_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      visuals_timer_=create_wall_timer(std::chrono::milliseconds(200),[this] {publishVisuals();},visuals_group_);
    }
    reference_sub_=create_subscription<global_planner::msg::ReferenceTrack>(
      "/global_planner/reference_track",rclcpp::QoS(1).reliable().transient_local(),
      [this](global_planner::msg::ReferenceTrack::ConstSharedPtr msg) {receiveReference(*msg);});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::QoS(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {receiveState(*msg);});
    grid_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap",rclcpp::QoS(1),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {receiveGrid(*msg);});
    enable_sub_=create_subscription<std_msgs::msg::Bool>("/dead_man_switch",rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (enabled_!=msg->data) {++generation_;reset_requested_=true;}
        enabled_=msg->data;
        enable_received_=Clock::now();
        if (!enabled_) {publishStop("Disabled");}
      });
    selected_sub_=create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd",1,
      [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        bool own=false;
        for (size_t i=0;i<own_count_;++i) {
          const auto & sent=own_requests_[i];
          own |= msg->header.stamp==sent.header.stamp && msg->drive.speed==sent.drive.speed &&
            msg->drive.steering_angle==sent.drive.steering_angle;
        }
        if (own) {
          const Command selected{msg->drive.speed,msg->drive.steering_angle};
          if (validCommand(selected) && rclcpp::Time(msg->header.stamp)>rclcpp::Time(selected_stamp_)) {
            previous_=selected;selected_at_=Clock::now();selected_stamp_=msg->header.stamp;
            ++selected_version_;
          }
        } else {
          manual_until_=Clock::now()+std::chrono::milliseconds(200);
          ++generation_;reset_requested_=true;
        }
      });
    watchdog_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this] {schedule();},watchdog_group_);
    watchdog_=create_wall_timer(std::chrono::milliseconds(5),[this] {
      std::lock_guard<std::mutex> lock(mutex_);
      if (busy_ && Clock::now()>active_deadline_) {++generation_;publishStop("Deadline");}
    },watchdog_group_);
    worker_=std::thread([this] {work();});
    RCLCPP_INFO(get_logger(),"Generated model: %s, wheelbase %.4f m, steering [%.4f, %.4f] rad",
      model_profile,wheelbase,steering_min,steering_max);
  }
  ~Node() override
  {
    {std::lock_guard<std::mutex> lock(mutex_);quit_=true;cv_.notify_one();}
    if (worker_.joinable()) {worker_.join();}
  }
private:
  struct Job {
    Pose ego;
    Command previous;
    double elapsed=.02;
    uint64_t id=0,grid_version=0,selected_version=0;
    bool allow=false,reset=false;
    builtin_interfaces::msg::Time state_stamp,grid_stamp;
    std::shared_ptr<const Reference> reference;
    std::shared_ptr<const Grid> grid;
    Clock::time_point release,deadline,state_received,grid_received;
  };
  bool fresh(const builtin_interfaces::msg::Time & stamp,double maximum) const
  {
    const double age=(now()-rclcpp::Time(stamp,get_clock()->get_clock_type())).seconds();
    return (stamp.sec!=0 || stamp.nanosec!=0) && age>=-.005 && age<=maximum;
  }
  tf2::Transform mapTransform(const std::string & frame,const builtin_interfaces::msg::Time & stamp)
  {
    tf2::Transform transform;transform.setIdentity();
    if (frame.empty()) {throw std::invalid_argument("Empty frame");}
    if (frame!="map") {
      auto msg=tf_.lookupTransform("map",frame,rclcpp::Time(stamp,get_clock()->get_clock_type()));
      tf2::fromMsg(msg.transform,transform);
    }
    return transform;
  }
  tf2::Transform poseTransform(const geometry_msgs::msg::Pose & pose) const
  {
    tf2::Transform transform;
    const auto & q=pose.orientation;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if (!std::isfinite(norm) || std::abs(norm-1)>1e-3 ||
      !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z)) {throw std::invalid_argument("Invalid pose");}
    tf2::fromMsg(pose,transform);
    if (transform.getBasis()[2][2]<std::cos(max_tilt_)) {
      throw std::invalid_argument("Nonplanar pose");
    }
    return transform;
  }
  void requirePlanar(const tf2::Transform & transform) const
  {
    for (int r=0;r<3;++r) {
      if (!std::isfinite(transform.getOrigin()[r])) {throw std::invalid_argument("Nonfinite TF");}
      for (int c=0;c<3;++c) {
        if (!std::isfinite(transform.getBasis()[r][c])) {throw std::invalid_argument("Nonfinite TF");}
      }
    }
    if (transform.getBasis()[2][2]<std::cos(max_tilt_)) {
      throw std::invalid_argument("Nonplanar composed transform");
    }
  }
  void receiveReference(const global_planner::msg::ReferenceTrack & msg)
  {
    try {
      if (msg.header.frame_id!="map" || msg.path.header.frame_id!="map") {
        throw std::invalid_argument("Reference must be in map");
      }
      std::shared_ptr<Reference> next;
      {std::lock_guard<std::mutex> lock(mutex_);
        next=reference_ ? std::make_shared<Reference>(*reference_) : std::make_shared<Reference>();}
      std::vector<local_planning::Point> points;
      std::vector<local_planning::TrackWidth> widths;
      for (const auto & p:msg.path.poses) {
        if (!p.header.frame_id.empty() && p.header.frame_id!="map") {
          throw std::invalid_argument("Inconsistent waypoint frame");
        }
        points.emplace_back(p.pose.position.x,p.pose.position.y,p.pose.position.z);
      }
      for (const auto & w:msg.widths) {widths.push_back({w.right_m,w.left_m});}
      std::array<std::vector<Pose>,2> boundaries;
      for (size_t side=0;side<2;++side) {
        const auto & loop=msg.legal_boundaries[side];
        if (!loop.poses.empty() && loop.header.frame_id!="map") {
          throw std::invalid_argument("Boundary must be in map");
        }
        for (const auto & p:loop.poses) {
          if (!p.header.frame_id.empty() && p.header.frame_id!="map") {
            throw std::invalid_argument("Inconsistent boundary frame");
          }
          boundaries[side].push_back({p.pose.position.x,p.pose.position.y,0});
        }
      }
      next->set(points,widths,boundaries);
      std::lock_guard<std::mutex> lock(mutex_);
      reference_=std::move(next);++generation_;reset_requested_=true;
    } catch (const std::exception & error) {
      std::lock_guard<std::mutex> lock(mutex_);
      reference_.reset();++generation_;publishStop(error.what());
    }
  }
  void receiveState(const nav_msgs::msg::Odometry & msg)
  {
    try {
      if (!fresh(msg.header.stamp,state_age_) || !std::isfinite(msg.twist.twist.linear.x) ||
        msg.child_frame_id!=vehicle_frame_) {throw std::invalid_argument("InvalidState");}
      const auto transform=mapTransform(msg.header.frame_id,msg.header.stamp)*poseTransform(msg.pose.pose);
      requirePlanar(transform);
      const auto rear=transform*tf2::Vector3(rear_offset_,0,0);
      const Pose pose{rear.x(),rear.y(),tf2::getYaw(transform.getRotation())};
      std::lock_guard<std::mutex> lock(mutex_);
      if (rclcpp::Time(msg.header.stamp)<=rclcpp::Time(state_stamp_)) {return;}
      ego_=pose;state_stamp_=msg.header.stamp;state_received_=Clock::now();have_state_=true;
    } catch (const std::exception &) {
      std::lock_guard<std::mutex> lock(mutex_);have_state_=false;
    }
  }
  void receiveGrid(const nav_msgs::msg::OccupancyGrid & msg)
  {
    try {
      if (!fresh(msg.header.stamp,grid_age_) || msg.info.width>1000 || msg.info.height>1000 ||
        msg.data.size()!=size_t(msg.info.width)*msg.info.height ||
        !std::isfinite(msg.info.resolution) || msg.info.resolution<.01) {
        throw std::invalid_argument("InvalidGrid");
      }
      auto transform=mapTransform(msg.header.frame_id,msg.header.stamp)*poseTransform(msg.info.origin);
      requirePlanar(transform);
      auto next=std::make_shared<Grid>();
      next->width=msg.info.width;next->height=msg.info.height;next->resolution=msg.info.resolution;
      next->origin={transform.getOrigin().x(),transform.getOrigin().y(),tf2::getYaw(transform.getRotation())};
      next->cells=msg.data;
      std::lock_guard<std::mutex> lock(mutex_);
      if (rclcpp::Time(msg.header.stamp)<=rclcpp::Time(grid_stamp_)) {return;}
      grid_=std::move(next);grid_stamp_=msg.header.stamp;grid_received_=Clock::now();++grid_version_;
    } catch (const std::exception &) {
      std::lock_guard<std::mutex> lock(mutex_);grid_.reset();++grid_version_;
    }
  }
  void publishCommand(Command command)
  {
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp=now();msg.header.frame_id=vehicle_frame_;
    msg.drive.speed=command.speed;msg.drive.steering_angle=command.steering;
    own_requests_[own_next_]=msg;own_next_=(own_next_+1)%own_requests_.size();
    own_count_=std::min(own_count_+1,own_requests_.size());command_pub_->publish(msg);
  }
  void publishStop(const std::string & reason)
  {
    publishCommand({});
    visual_approved_=false;visual_reason_=reason;
    if (path_pub_->get_subscription_count()) {
      nav_msgs::msg::Path empty;empty.header.frame_id="map";empty.header.stamp=now();
      path_pub_->publish(empty);
    }
    std_msgs::msg::String status;status.data=reason;reason_pub_->publish(status);
  }
  void publishVisuals()
  {
    std::shared_ptr<const Reference> reference,scene_reference;
    std::shared_ptr<const Scene> scene;
    std::optional<Cycle> result;
    std::vector<Pose> route;
    Pose ego;
    bool state_valid=false,approved=false;
    std::string reason;
    builtin_interfaces::msg::Time grid_stamp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      reference=reference_;ego=ego_;reason=visual_reason_;grid_stamp=grid_stamp_;
      const auto time=Clock::now();
      state_valid=have_state_ && fresh(state_stamp_,state_age_) &&
        std::chrono::duration<double>(time-state_received_).count()<=state_age_;
      const bool grid_valid=grid_ && fresh(grid_stamp_,grid_age_) &&
        std::chrono::duration<double>(time-grid_received_).count()<=grid_age_;
      if (grid_valid && reference && visual_scene_reference_==reference &&
        visual_scene_ && visual_scene_->version()==grid_version_) {
        scene=visual_scene_;scene_reference=visual_scene_reference_;
      }
      if (state_valid && scene && visual_cycle_ &&
        std::chrono::duration<double>(time-visual_cycle_time_).count()<=state_age_ &&
        visual_cycle_->problem.scene_version==grid_version_ &&
        visual_cycle_->problem.track_version==reference->version()) {
        result=visual_cycle_;route=visual_route_;
        approved=visual_approved_ && enabled_ &&
          time-enable_received_<std::chrono::milliseconds(150) && time>=manual_until_;
      }
    }
    if (!approved && reason=="Approved") {reason="NoCurrentApprovedCycle";}
    // Build and serialize debug messages outside the command/publication mutex.
    std_msgs::msg::Header header;header.frame_id="map";header.stamp=now();
    if (reference!=drawn_reference_) {
      if (reference) {track_pub_->publish(visualization::track(*reference,header));}
      else {
        visualization::Markers clear;visualization_msgs::msg::Marker marker;
        marker.header=header;marker.action=visualization_msgs::msg::Marker::DELETEALL;
        clear.markers.push_back(marker);track_pub_->publish(clear);
      }
      drawn_reference_=reference;
    }
    if (search_pub_->get_subscription_count()) {
      if (scene) {
        auto observed=header;observed.stamp=grid_stamp;
        search_pub_->publish(visualization::searchSpace(*scene,observed));
      } else {search_pub_->publish(visualization::emptySearchSpace(header));}
    }
    if (visuals_pub_->get_subscription_count()) {
      visuals_pub_->publish(visualization::cycle(result ? &*result : nullptr,route,
        ego,state_valid,approved,reason,header));
    }
  }
  void schedule()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto release=Clock::now();++generation_;
    const auto ros_time=now().nanoseconds();
    if (ros_time<last_ros_time_) {
      last_ros_time_=ros_time;have_state_=false;grid_.reset();++grid_version_;
      state_stamp_=builtin_interfaces::msg::Time();grid_stamp_=builtin_interfaces::msg::Time();
      previous_={};selected_stamp_=builtin_interfaces::msg::Time();
      selected_at_={};own_count_=own_next_=0;
      reset_requested_=true;publishStop("ClockReset");return;
    }
    last_ros_time_=ros_time;
    if (busy_ || !have_state_ || !reference_ || !grid_ ||
      !fresh(state_stamp_,state_age_) || !fresh(grid_stamp_,grid_age_) ||
      std::chrono::duration<double>(release-state_received_).count()>state_age_ ||
      std::chrono::duration<double>(release-grid_received_).count()>grid_age_ || release<manual_until_) {
      publishStop(busy_ ? "Busy" : "NoFreshSceneOrManual");return;
    }
    const double elapsed=selected_at_==Clock::time_point{} ? .02 :
      std::chrono::duration<double>(release-selected_at_).count();
    if (selected_at_==Clock::time_point{} || elapsed>.06) {
      reset_requested_=true;publishStop("NoSelectedCommandHistory");return;
    }
    Job job;
    job.ego=ego_;job.previous=previous_;job.elapsed=std::clamp(elapsed,.001,.06);
    job.id=generation_;job.grid_version=grid_version_;job.reference=reference_;job.grid=grid_;
    job.selected_version=selected_version_;
    job.state_stamp=state_stamp_;job.grid_stamp=grid_stamp_;
    job.state_received=state_received_;job.grid_received=grid_received_;
    job.release=release;job.deadline=release+std::chrono::milliseconds(20);
    job.allow=enabled_ && release-enable_received_<std::chrono::milliseconds(150);
    job.reset=reset_requested_;reset_requested_=false;
    active_deadline_=job.deadline;job_=std::move(job);busy_=true;cv_.notify_one();
  }
  void work()
  {
    std::shared_ptr<const Reference> cached_reference;
    std::shared_ptr<Scene> scene;
    uint64_t cached_grid=0;
    for (;;) {
      Job job;
      {std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock,[this] {return quit_ || job_.has_value();});
        if (quit_) {return;}
        job=std::move(*job_);job_.reset();}
      Cycle result;
      try {
        if (job.reset) {controller_.reset();}
        if (!scene || cached_grid!=job.grid_version || cached_reference!=job.reference) {
          scene=std::make_shared<Scene>(*job.grid,*job.reference,job.grid_version);
          cached_reference=job.reference;cached_grid=job.grid_version;
        }
        if (stall_ms_) {std::this_thread::sleep_for(std::chrono::milliseconds(stall_ms_));}
        result=controller_.cycle(job.ego,job.previous,job.elapsed,*job.reference,*scene,job.deadline);
      } catch (const std::exception & error) {result.validation.reason=error.what();}
      std::lock_guard<std::mutex> lock(mutex_);busy_=false;
      const bool approved=result.validation.accepted && Clock::now()<=job.deadline &&
        job.id==generation_ && grid_version_==job.grid_version && reference_==job.reference &&
        selected_version_==job.selected_version &&
        have_state_ && fresh(job.state_stamp,state_age_) && fresh(job.grid_stamp,grid_age_) &&
        std::chrono::duration<double>(Clock::now()-job.state_received).count()<=state_age_ &&
        std::chrono::duration<double>(Clock::now()-job.grid_received).count()<=grid_age_ &&
        job.allow && enabled_ && Clock::now()-enable_received_<std::chrono::milliseconds(150) &&
        Clock::now()>=manual_until_;
      std::string reason=result.validation.reason;
      if (!approved && result.validation.accepted) {reason="PublicationGate";}
      if (!enabled_) {reason="Disabled";}
      else if (Clock::now()-enable_received_>=std::chrono::milliseconds(150)) {reason="EnableTimeout";}
      if (approved) {
        publishCommand(result.solution.commands[0]);
        if (path_pub_->get_subscription_count()) {
          std_msgs::msg::Header header;header.frame_id="map";header.stamp=now();
          path_pub_->publish(visualization::path(result.solution,header));
        }
      } else {publishStop(reason);}
      std_msgs::msg::String status;status.data=approved ? "Approved" :
        reason;reason_pub_->publish(status);
      if (visuals_enabled_) {
        visual_approved_=approved;visual_reason_=status.data;
        visual_scene_=scene;visual_scene_reference_=cached_reference;
        if (visuals_pub_->get_subscription_count()) {
          visual_cycle_=result;visual_cycle_time_=Clock::now();
          visual_route_=controller_.route();
        } else {visual_cycle_.reset();visual_route_.clear();}
      }
      std_msgs::msg::Float64MultiArray metrics;
      metrics.data={std::chrono::duration<double,std::milli>(Clock::now()-job.release).count(),
        result.prepare_ms,result.solution.solve_ms,result.validation_ms,double(approved),
        result.validation.minimum_clearance,double(job.id),
        (now()-rclcpp::Time(job.state_stamp,get_clock()->get_clock_type())).seconds(),
        (now()-rclcpp::Time(job.grid_stamp,get_clock()->get_clock_type())).seconds()};
      diagnostics_pub_->publish(metrics);
    }
  }
  Controller controller_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  bool quit_=false,busy_=false,enabled_=false,have_state_=false,reset_requested_=false;
  Pose ego_;
  Command previous_;
  std::string vehicle_frame_;
  double state_age_,grid_age_,rear_offset_,max_tilt_;
  int stall_ms_;
  int64_t last_ros_time_=0;
  uint64_t generation_=0,grid_version_=0,selected_version_=0;
  builtin_interfaces::msg::Time state_stamp_,grid_stamp_,selected_stamp_;
  std::array<ackermann_msgs::msg::AckermannDriveStamped,32> own_requests_;
  size_t own_count_=0,own_next_=0;
  Clock::time_point selected_at_,manual_until_,active_deadline_,enable_received_,state_received_,grid_received_;
  std::optional<Job> job_;
  std::shared_ptr<const Reference> reference_;
  std::shared_ptr<const Grid> grid_;
  bool visuals_enabled_=true,visual_approved_=false;
  std::string visual_reason_="NoInputs";
  std::shared_ptr<const Reference> visual_scene_reference_,drawn_reference_;
  std::shared_ptr<const Scene> visual_scene_;
  std::optional<Cycle> visual_cycle_;
  std::vector<Pose> visual_route_;
  Clock::time_point visual_cycle_time_;
  rclcpp::CallbackGroup::SharedPtr visuals_group_;
  rclcpp::TimerBase::SharedPtr visuals_timer_;
  rclcpp::Publisher<visualization::Markers>::SharedPtr track_pub_,visuals_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr search_pub_;
  rclcpp::CallbackGroup::SharedPtr watchdog_group_;
  rclcpp::TimerBase::SharedPtr timer_,watchdog_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<global_planner::msg::ReferenceTrack>::SharedPtr reference_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr selected_sub_;
};
}  // namespace mpc

int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  auto node=std::make_shared<mpc::Node>();
  // Input preparation and visualization must leave a thread available for the watchdog.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);
  executor.add_node(node);executor.spin();rclcpp::shutdown();return 0;
}
