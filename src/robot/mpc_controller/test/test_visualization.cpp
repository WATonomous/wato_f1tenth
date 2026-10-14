#include "fixtures.hpp"
#include "mpc_controller/visualization.hpp"
#include <gtest/gtest.h>

namespace {
std_msgs::msg::Header header()
{std_msgs::msg::Header h;h.frame_id="map";h.stamp.sec=123;return h;}
const visualization_msgs::msg::Marker & find(const mpc::visualization::Markers & markers,
  const std::string & name)
{
  for (const auto & marker:markers.markers) {if (marker.ns==name) {return marker;}}
  throw std::runtime_error("Missing marker: "+name);
}
}

TEST(Visualization, RacelineAndLegalBoundariesAreFlatClosedAndStamped)
{
  mpc::Reference reference;circleTrack(reference);
  const auto markers=mpc::visualization::track(reference,header());
  ASSERT_EQ(markers.markers.size(),4);
  for (size_t i=1;i<markers.markers.size();++i) {
    const auto & m=markers.markers[i];
    ASSERT_GT(m.points.size(),4);
    EXPECT_EQ(m.header,header());EXPECT_DOUBLE_EQ(m.pose.orientation.w,1);
    EXPECT_NEAR(m.points.front().x,m.points.back().x,1e-9);
    EXPECT_NEAR(m.points.front().y,m.points.back().y,1e-9);
    for (auto p:m.points) {EXPECT_LT(p.z,.1);}
  }
}

TEST(Visualization, SearchGridUsesActualMaskAndPreservesNativeAxes)
{
  mpc::Reference reference;circleTrack(reference);
  auto grid=freeGrid();grid.origin.yaw=.2;grid.cells[45*grid.width+35]=-1;
  const mpc::Scene scene(grid,reference,7);
  const auto message=mpc::visualization::searchSpace(scene,header());
  EXPECT_EQ(message.header,header());EXPECT_EQ(message.info.width,grid.width);
  EXPECT_EQ(message.info.height,grid.height);EXPECT_NEAR(message.info.resolution,grid.resolution,1e-7);
  EXPECT_DOUBLE_EQ(message.info.origin.position.x,grid.origin.x);
  EXPECT_DOUBLE_EQ(message.info.origin.position.y,grid.origin.y);
  EXPECT_NEAR(message.info.origin.orientation.z,std::sin(.1),1e-12);
  ASSERT_EQ(message.data.size(),scene.safeMask().size());
  for (size_t i=0;i<message.data.size();++i) {EXPECT_EQ(message.data[i],scene.safeMask()[i] ? 0 : 100);}
  EXPECT_EQ(mpc::visualization::emptySearchSpace(header()).data,std::vector<int8_t>{-1});
}

TEST(Visualization, ProposalApprovalAndCorridorFrameAreExplicit)
{
  mpc::Cycle result;result.solution.status=0;result.solution.frame={2,3,.7};
  result.problem.frame=result.solution.frame;result.problem.track_version=1;
  for (auto & faces:result.problem.corridor) {faces={{{1,0,1},{-1,0,1},{0,1,2},{0,-1,0}}};}
  auto markers=mpc::visualization::cycle(&result,{{2,3,0},{3,4,0}}, {},true,false,"RolloutDefect",header());
  const auto & proposal=find(markers,"unapproved_proposal");
  EXPECT_FLOAT_EQ(proposal.color.r,1);EXPECT_EQ(proposal.points.size(),mpc::horizon+1);
  EXPECT_NEAR(proposal.points[0].x,2,1e-9);EXPECT_NEAR(proposal.points[0].y,3,1e-9);
  const auto & corridors=find(markers,"solver_corridors_covering_center");
  ASSERT_EQ(corridors.points.size(),8*(mpc::horizon+1));
  for (size_t i=0;i<corridors.points.size();++i) {
    const auto p=corridors.points[i];
    const auto local=mpc::toLocal({p.x,p.y,0},result.problem.frame);
    EXPECT_LE(std::abs(local.x),1+1e-9);EXPECT_GE(local.y,-1e-9);EXPECT_LE(local.y,2+1e-9);
    EXPECT_NEAR(std::abs(local.x),i<8 ? 1 : .998,1e-9);
  }
  markers=mpc::visualization::cycle(&result,{}, {},true,true,"Approved",header());
  EXPECT_FLOAT_EQ(find(markers,"approved_proposal").color.g,1);
  result.solution.states[5][0]=NAN;
  EXPECT_TRUE(mpc::visualization::path(result.solution,header()).poses.empty());
}

TEST(Visualization, FootprintAndEnvelopeUseCoveringCenterAndExpire)
{
  const mpc::Pose ego{2,3,std::acos(-1.)/2};
  const auto markers=mpc::visualization::cycle(nullptr,{},ego,true,false,"Disabled",header());
  const auto center=mpc::coveringCenter(ego);
  const auto & body=find(markers,"vehicle_footprint");
  EXPECT_NEAR(body.pose.position.x,center.x,1e-12);EXPECT_NEAR(body.pose.position.y,center.y,1e-12);
  EXPECT_DOUBLE_EQ(body.scale.x,mpc::front_extent+mpc::rear_extent);
  const auto & circle=find(markers,"covering_circle_plus_clearance");
  for (auto p:circle.points) {
    EXPECT_NEAR(std::hypot(p.x-center.x,p.y-center.y),mpc::coveringRadius()+mpc::clearance+.005,1e-12);
  }
  EXPECT_GT(body.lifetime.nanosec,0);
  const auto stale=mpc::visualization::cycle(nullptr,{},ego,false,false,"NoFreshSceneOrManual",header());
  EXPECT_EQ(stale.markers.size(),2);EXPECT_EQ(stale.markers[0].action,3);
  EXPECT_NE(find(stale,"command_status").text.find("NO FRESH STATE"),std::string::npos);
}
