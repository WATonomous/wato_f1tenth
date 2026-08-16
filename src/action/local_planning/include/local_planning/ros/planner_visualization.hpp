#ifndef LOCAL_PLANNING_ROS_PLANNER_VISUALIZATION_HPP
#define LOCAL_PLANNING_ROS_PLANNER_VISUALIZATION_HPP

#include "local_planning/planning/local_planner.hpp"
#include "local_planning/reference/raceline_reference.hpp"
#include "local_planning/state/racing_state_machine.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace local_planning
{

struct PlannerVisualizationConfig
{
  std::string map_frame;
  bool publish_projection_markers = true;
  double vehicle_full_width_m = 0.0;
  double compat_heading_rad = 0.0;
};

// One maneuver family exactly as ManeuverBuilder returned it: before collision,
// track-bounds, or velocity filtering, and before selection ever sees it.
struct CandidateFamily
{
  CandidateSource source = CandidateSource::NONE;
  std::vector<ManeuverCandidate> candidates;
};

class PlannerVisualization
{
public:
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using MarkerPublisher = rclcpp::Publisher<MarkerArray>;

  PlannerVisualization(
    const RacelineReference & reference,
    PlannerVisualizationConfig config);

  void publishCandidates(
    const LocalPlanResult & result,
    const rclcpp::Time & stamp,
    MarkerPublisher & publisher) const;
  // Generated paths, unfiltered, one namespace per family. A raw view of what
  // ManeuverBuilder produced: nothing here has been collision checked, bounds
  // checked, or ranked, and a drawn path is not a drivable one. publishCandidates
  // is the opposite view -- only what survived to selection, and only on the
  // cycles that reached it. Callers decide which families belong on the picture;
  // an empty list clears the topic.
  void publishAllCandidates(
    const std::vector<CandidateFamily> & families,
    const rclcpp::Time & stamp,
    MarkerPublisher & publisher) const;
  void publishProjection(
    const Odometry & odom,
    const TacticalState & state,
    const rclcpp::Time & stamp,
    MarkerPublisher & publisher) const;
  void publishTrackBounds(
    const rclcpp::Time & stamp,
    MarkerPublisher & publisher) const;

private:
  const RacelineReference & reference_;
  const PlannerVisualizationConfig config_;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_ROS_PLANNER_VISUALIZATION_HPP
