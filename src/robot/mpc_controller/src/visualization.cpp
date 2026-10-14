#include "mpc_controller/visualization.hpp"
#include <iomanip>
#include <sstream>

namespace mpc::visualization {
namespace {
using Marker = visualization_msgs::msg::Marker;
geometry_msgs::msg::Point point(Pose p, double z=.04)
{
  geometry_msgs::msg::Point result;
  result.x=p.x;result.y=p.y;result.z=z;return result;
}
Marker line(const std::string & name, const std_msgs::msg::Header & header,
  float r, float g, float b, double width=.025)
{
  Marker marker;marker.header=header;marker.ns=name;marker.id=0;
  marker.type=Marker::LINE_STRIP;marker.action=Marker::ADD;
  marker.pose.orientation.w=1;marker.scale.x=width;
  marker.color.r=r;marker.color.g=g;marker.color.b=b;marker.color.a=1;
  return marker;
}
void clear(Markers & markers, const std_msgs::msg::Header & header)
{
  Marker marker;marker.header=header;marker.action=Marker::DELETEALL;
  markers.markers.push_back(marker);
}
std::vector<Pose> corners(const std::array<Face,4> & faces)
{
  std::vector<Pose> result;
  for (size_t i=0;i<faces.size();++i) {
    for (size_t j=i+1;j<faces.size();++j) {
      const auto a=faces[i],b=faces[j];
      const double determinant=a.x*b.y-a.y*b.x;
      if (std::abs(determinant)<1e-9) {continue;}
      const Pose p{(a.bound*b.y-a.y*b.bound)/determinant,
        (a.x*b.bound-a.bound*b.x)/determinant,0};
      if (!finite(p)) {continue;}
      if (std::all_of(faces.begin(),faces.end(),[&](Face f) {
        return f.x*p.x+f.y*p.y<=f.bound+1e-8;
      })) {result.push_back(p);}
    }
  }
  if (result.size()!=4) {return {};}
  Pose center;
  for (auto p:result) {center.x+=p.x/4;center.y+=p.y/4;}
  std::sort(result.begin(),result.end(),[&](Pose a,Pose b) {
    return std::atan2(a.y-center.y,a.x-center.x)<std::atan2(b.y-center.y,b.x-center.x);
  });
  return result;
}
}  // namespace

Markers track(const Reference & reference,const std_msgs::msg::Header & header)
{
  Markers markers;clear(markers,header);
  auto raceline=line("raceline",header,.3F,.65F,1.F,.025);
  raceline.color.a=.7F;
  const auto & geometry=reference.geometry();
  const int count=std::min(40000,std::max(4,int(std::ceil(geometry.totalLength()/.05))));
  for (int i=0;i<=count;++i) {
    const auto p=geometry.sampleAtS(geometry.totalLength()*i/count);
    raceline.points.push_back(point({p.x,p.y,0}));
  }
  markers.markers.push_back(std::move(raceline));
  for (int side=0;side<2;++side) {
    auto boundary=line(side==0 ? "legal_boundary_0" : "legal_boundary_1",
      header,.65F,.7F,.75F,.015);
    boundary.color.a=.55F;
    for (auto p:reference.edges()[side]) {boundary.points.push_back(point(p));}
    if (!boundary.points.empty()) {boundary.points.push_back(boundary.points.front());}
    markers.markers.push_back(std::move(boundary));
  }
  return markers;
}

nav_msgs::msg::OccupancyGrid searchSpace(const Scene & scene,const std_msgs::msg::Header & header)
{
  nav_msgs::msg::OccupancyGrid message;message.header=header;
  const auto & grid=scene.grid();
  message.info.width=grid.width;message.info.height=grid.height;message.info.resolution=grid.resolution;
  message.info.origin.position=point(grid.origin,.01);
  message.info.origin.orientation.z=std::sin(grid.origin.yaw/2);
  message.info.origin.orientation.w=std::cos(grid.origin.yaw/2);
  message.data.reserve(scene.safeMask().size());
  for (auto safe:scene.safeMask()) {message.data.push_back(safe ? 0 : 100);}
  return message;
}

nav_msgs::msg::OccupancyGrid emptySearchSpace(const std_msgs::msg::Header & header)
{
  nav_msgs::msg::OccupancyGrid message;message.header=header;
  message.info.width=message.info.height=1;message.info.resolution=1;
  message.info.origin.orientation.w=1;message.data={-1};return message;
}

nav_msgs::msg::Path path(const Solution & solution,const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path message;message.header=header;
  if (solution.status<0 || !finite(solution.frame)) {return message;}
  for (auto state:solution.states) {
    if (!std::all_of(state.begin(),state.end(),[](double v) {return std::isfinite(v);})) {
      message.poses.clear();return message;
    }
    if (std::hypot(state[0],state[1])>2*horizon*dt*speed_cap+1) {
      message.poses.clear();return message;
    }
    const auto p=toMap({state[0],state[1],state[2]},solution.frame);
    if (!finite(p)) {message.poses.clear();return message;}
    geometry_msgs::msg::PoseStamped pose;pose.header=header;pose.pose.position=point(p,0);
    pose.pose.orientation.z=std::sin(p.yaw/2);pose.pose.orientation.w=std::cos(p.yaw/2);
    message.poses.push_back(pose);
  }
  return message;
}

Markers cycle(const Cycle * result,const std::vector<Pose> & route,Pose ego,
  bool state_valid,bool approved,const std::string & reason,const std_msgs::msg::Header & header)
{
  Markers markers;clear(markers,header);
  if (result) {
    const auto candidate=path(result->solution,header);
    if (!candidate.poses.empty()) {
      auto proposal=line(approved ? "approved_proposal" : "unapproved_proposal",header,
        approved ? .3F : 1.F,approved ? 1.F : .35F,.25F,.05);
      for (const auto & p:candidate.poses) {
        auto position=p.pose.position;position.z=.06;proposal.points.push_back(position);
      }
      markers.markers.push_back(std::move(proposal));
    }
    if (result->problem.track_version!=0) {
      auto seed=line("seed_route_rear_axle",header,.85F,.7F,.35F,.012);
      seed.color.a=.55F;
      for (auto p:route) {if (finite(p)) {seed.points.push_back(point(p,.045));}}
      markers.markers.push_back(std::move(seed));
      auto warm=line("warm_start_rear_axle",header,.65F,.65F,.65F,.01);
      warm.color.a=.3F;
      for (auto x:result->problem.seed_states) {
        if (std::isfinite(x[0]) && std::isfinite(x[1]) && std::isfinite(x[2])) {
          warm.points.push_back(point(toMap({x[0],x[1],x[2]},result->problem.frame),.035));
        }
      }
      markers.markers.push_back(std::move(warm));
      auto corridors=line("solver_corridors_covering_center",header,.35F,.7F,.75F,.008);
      corridors.type=Marker::LINE_LIST;corridors.color.a=.12F;
      for (int stage=0;stage<=horizon;++stage) {
        auto faces=result->problem.corridor[stage];
        for (auto & face:faces) {face=solverCorridorFace(face,result->problem.seed_states[stage],stage);}
        const auto polygon=corners(faces);
        for (size_t i=0;i<polygon.size();++i) {
          corridors.points.push_back(point(toMap(polygon[i],result->problem.frame),.025));
          corridors.points.push_back(point(toMap(polygon[(i+1)%polygon.size()],result->problem.frame),.025));
        }
      }
      markers.markers.push_back(std::move(corridors));
    }
  }
  if (state_valid && finite(ego)) {
    auto body=line("vehicle_footprint",header,.85F,.9F,.95F);
    body.type=Marker::CUBE;body.pose.position=point(coveringCenter(ego),.1);
    body.pose.orientation.z=std::sin(ego.yaw/2);body.pose.orientation.w=std::cos(ego.yaw/2);
    body.scale.x=front_extent+rear_extent;body.scale.y=2*half_width;body.scale.z=.16;
    body.color.a=.85F;markers.markers.push_back(std::move(body));
    auto safety=line("covering_circle_plus_clearance",header,1.F,.65F,.25F,.01);
    safety.color.a=.55F;
    const auto center=coveringCenter(ego);const double radius=coveringRadius()+clearance+.005;
    for (int i=0;i<=64;++i) {
      const double angle=i*2*std::acos(-1.)/64;
      safety.points.push_back(point({center.x+radius*std::cos(angle),center.y+radius*std::sin(angle),0},.12));
    }
    markers.markers.push_back(std::move(safety));
  }
  if (finite(ego)) {
    auto status=line("command_status",header,approved ? .15F : 1.F,approved ? 1.F : .35F,.15F);
    status.type=Marker::TEXT_VIEW_FACING;status.pose.position=point(ego,.75);status.scale.z=.18;
    std::ostringstream text;text<<(approved ? "APPROVED" : reason);
    if (result) {text<<std::fixed<<std::setprecision(2)<<"\nMPC core "<<result->total_ms
      <<" ms | solve "<<result->solution.solve_ms<<" ms | status "<<result->solution.status;}
    if (!state_valid) {text<<"\nNO FRESH STATE";}
    status.text=text.str();markers.markers.push_back(std::move(status));
  }
  for (size_t i=1;i<markers.markers.size();++i) {
    markers.markers[i].lifetime.sec=0;markers.markers[i].lifetime.nanosec=400000000;
  }
  return markers;
}
}  // namespace mpc::visualization
