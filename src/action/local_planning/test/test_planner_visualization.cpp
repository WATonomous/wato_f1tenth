#include "local_planning/ros/planner_visualization.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace local_planning
{
namespace
{

using MarkerArray = visualization_msgs::msg::MarkerArray;
using namespace std::chrono_literals;

std::vector<Point> circleLine(double radius, int count)
{
  constexpr double kPi = 3.14159265358979323846;
  std::vector<Point> points;
  for (int i = 0; i < count; ++i) {
    const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(count);
    points.emplace_back(radius * std::cos(angle), radius * std::sin(angle), 3.0);
  }
  return points;
}

class MarkerReceiver
{
public:
  explicit MarkerReceiver(const std::string & name)
  : node_(std::make_shared<rclcpp::Node>(name)),
    publisher_(node_->create_publisher<MarkerArray>(name + "/markers", 10)),
    subscription_(node_->create_subscription<MarkerArray>(
        name + "/markers", 10,
        [this](MarkerArray::SharedPtr message) {last_ = std::move(message);})),
    executor_()
  {
    executor_.add_node(node_);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (publisher_->get_subscription_count() == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    if (publisher_->get_subscription_count() == 0) {
      throw std::runtime_error("marker test publisher did not discover its subscription");
    }
  }

  PlannerVisualization::MarkerPublisher & publisher() {return *publisher_;}
  rclcpp::Time now() const {return node_->now();}

  MarkerArray::SharedPtr take()
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!last_ && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    return std::move(last_);
  }

  bool receivesWithin(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (!last_ && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    return static_cast<bool>(last_);
  }

private:
  rclcpp::Node::SharedPtr node_;
  PlannerVisualization::MarkerPublisher::SharedPtr publisher_;
  rclcpp::Subscription<MarkerArray>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  MarkerArray::SharedPtr last_;
};

class PlannerVisualizationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    const std::string log_directory = "/tmp/wato_f1tenth_visualization_test_logs";
    std::filesystem::create_directories(log_directory);
    setenv("ROS_LOG_DIR", log_directory.c_str(), 1);
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(PlannerVisualizationTest, PublishesCandidatePoolSelectionAndTerminal)
{
  RacelineReference reference;
  PlannerVisualization visualization(reference, {"map", true, 0.4, 1.05});
  MarkerReceiver receiver("candidate_visualization_test");
  LocalPlanResult result;
  result.pool.resize(2);
  CurveSample first;
  first.x = 1.0;
  first.y = 2.0;
  CurveSample second;
  second.x = 3.0;
  second.y = 4.0;
  result.pool[0].path.push_back(first);
  result.pool[1].path.push_back(second);
  result.selected_index = 1;

  visualization.publishCandidates(result, receiver.now(), receiver.publisher());
  const auto message = receiver.take();

  ASSERT_NE(message, nullptr);
  ASSERT_EQ(message->markers.size(), 4U);
  EXPECT_EQ(message->markers[0].action, visualization_msgs::msg::Marker::DELETEALL);
  EXPECT_EQ(message->markers[1].ns, "candidate_paths");
  EXPECT_DOUBLE_EQ(message->markers[1].scale.x, 0.025);
  EXPECT_EQ(message->markers[2].id, 1);
  EXPECT_DOUBLE_EQ(message->markers[2].scale.x, 0.08);
  ASSERT_EQ(message->markers[2].points.size(), 1U);
  EXPECT_DOUBLE_EQ(message->markers[2].points[0].x, 3.0);
  EXPECT_EQ(message->markers[3].ns, "selected_terminal_offset");
  EXPECT_DOUBLE_EQ(message->markers[3].pose.position.y, 4.0);
}

TEST_F(PlannerVisualizationTest, ProjectionHonorsEnableAndValidityAndPublishesFiveMarkers)
{
  RacelineReference reference;
  ASSERT_TRUE(reference.setRacingLine(circleLine(5.0, 48)));
  MarkerReceiver receiver("projection_visualization_test");
  Odometry odom;
  odom.position = Point(5.2, 0.1);
  odom.heading = 0.2;
  TacticalState state;
  state.intent = PlannerIntent::MERGE;
  state.ego_s = 0.2;
  state.ego_d = -0.25;
  state.heading_error_rad = 0.12;
  state.raceline_compatible = false;
  state.ego_seed_was_stale = true;

  PlannerVisualization disabled(reference, {"map", false, 0.4, 1.05});
  disabled.publishProjection(odom, state, receiver.now(), receiver.publisher());
  EXPECT_FALSE(receiver.receivesWithin(100ms));

  PlannerVisualization visualization(reference, {"map", true, 0.4, 1.05});
  visualization.publishProjection(odom, state, receiver.now(), receiver.publisher());
  const auto message = receiver.take();

  ASSERT_NE(message, nullptr);
  ASSERT_EQ(message->markers.size(), 5U);
  EXPECT_EQ(message->markers[0].type, visualization_msgs::msg::Marker::SPHERE);
  EXPECT_EQ(message->markers[1].type, visualization_msgs::msg::Marker::LINE_STRIP);
  ASSERT_EQ(message->markers[1].points.size(), 2U);
  EXPECT_EQ(message->markers[2].type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_EQ(message->markers[3].type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_EQ(message->markers[4].ns, "ego_projection_text");
  EXPECT_NE(message->markers[4].text.find("MERGE SEED-STALE"), std::string::npos);

  RacelineReference invalid_reference;
  PlannerVisualization invalid(invalid_reference, {"map", true, 0.4, 1.05});
  invalid.publishProjection(odom, state, receiver.now(), receiver.publisher());
  EXPECT_FALSE(receiver.receivesWithin(100ms));
}

TEST_F(PlannerVisualizationTest, TrackBoundsPublishTwoClosedRawLines)
{
  RacelineReference reference;
  const auto points = circleLine(5.0, 48);
  ASSERT_TRUE(reference.setRacingLine(points));
  std::vector<TrackWidth> widths(points.size(), TrackWidth{1.2, 1.5});
  ASSERT_TRUE(reference.setTrackWidths(widths, 0.2, 0.05, 0.25));
  PlannerVisualization visualization(reference, {"map", true, 0.4, 1.05});
  MarkerReceiver receiver("track_bounds_visualization_test");

  visualization.publishTrackBounds(receiver.now(), receiver.publisher());
  const auto message = receiver.take();

  ASSERT_NE(message, nullptr);
  ASSERT_EQ(message->markers.size(), 2U);
  EXPECT_EQ(message->markers[0].ns, "track_bounds_raw");
  EXPECT_EQ(message->markers[1].ns, "track_bounds_raw");
  for (const auto & marker : message->markers) {
    ASSERT_EQ(marker.points.size(), reference.widthSampleCount() + 1U);
    EXPECT_NEAR(marker.points.front().x, marker.points.back().x, 1e-12);
    EXPECT_NEAR(marker.points.front().y, marker.points.back().y, 1e-12);
  }
}

}  // namespace
}  // namespace local_planning
