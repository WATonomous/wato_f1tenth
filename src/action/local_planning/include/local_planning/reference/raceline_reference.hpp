#ifndef LOCAL_PLANNING_REFERENCE_RACELINE_REFERENCE_HPP
#define LOCAL_PLANNING_REFERENCE_RACELINE_REFERENCE_HPP

#include "local_planning/core/types.hpp"

#include <cstddef>
#include <vector>

namespace local_planning
{

struct ProjectionConfig
{
  double seed_window_m = 2.0;           // +/- arc length around seed; keep >> max seed step
  double tangent_tolerance_rad = 1.2;   // reject wrong loop branch by heading
  double max_plausible_offset_m = 3.0;  // stale seed if nearest foot farther than this
};

struct Projection
{
  double s = 0.0;
  double d = 0.0;   // signed lateral offset, positive left

  bool seed_was_stale = false;         // fell back to full-loop search
  bool heading_check_relaxed = false;  // tangent tolerance was widened
};

struct SustainableBounds
{
  double right_magnitude = 0.0;
  double left_magnitude = 0.0;
};

struct WidthLookupSample
{
  double s = 0.0;
  SustainableBounds raw;
};

struct TrackWidth
{
  double right_m = 0.0;
  double left_m = 0.0;
};

// Periodic cubic-spline raceline; projection is locally seeded (caller-owned seed).
class RacelineReference
{
public:
  bool setRacingLine(const std::vector<Point> & points);
  void setProjectionConfig(const ProjectionConfig & config) {projection_config_ = config;}
  const ProjectionConfig & projectionConfig() const {return projection_config_;}

  bool valid() const {return valid_;}
  double totalLength() const {return total_length_m_;}
  std::size_t waypointCount() const {return points_.size();}

  bool setTrackWidths(
    const std::vector<TrackWidth> & widths,
    double spacing_m);
  void clearTrackWidths();
  bool trackWidthsValid() const {return track_widths_valid_;}
  SustainableBounds rawBounds(double s) const;
  WidthLookupSample widthSample(std::size_t index) const;
  std::size_t widthSampleCount() const {return raw_left_m_.size();}

  ReferenceGeometrySample sampleAtS(double s) const;
  double velocityAtS(double s) const;
  Point toCartesian(double s, double d) const;

  Projection project(const Point & p, double heading, double seed_s) const;
  Projection project(const Point & p, double seed_s) const;  // no heading check
  Projection projectGlobal(const Point & p, double heading, double tolerance_rad) const;

  double wrapS(double s) const;
  double deltaS(double from_s, double to_s) const;  // wrapped signed delta, + = ahead

private:
  struct SplineSegment
  {
    // Cubic in t = s - segment_start, valid over t in [0, length].
    double a = 0.0;   // value at t = 0
    double b = 0.0;   // first derivative at t = 0
    double c = 0.0;   // second derivative at t = 0, halved
    double d = 0.0;   // third derivative, sixthed
  };

  std::size_t segmentAt(double s_wrapped, double & t) const;
  double velocityOnSegment(std::size_t i, double t) const;

  bool scanSegment(
    const Point & p,
    double heading,
    double tolerance_rad,
    std::size_t segment,
    Projection & best,
    double & best_dist_sq) const;

  Projection searchArc(
    const Point & p,
    double heading,
    double tolerance_rad,
    double start_s,
    double length_m,
    bool & found,
    double & best_dist_sq) const;

  Projection searchAllSegments(
    const Point & p,
    double heading,
    double tolerance_rad,
    bool & found) const;

  double refineOnSegment(const Point & p, std::size_t segment, double t_initial) const;

  bool valid_ = false;
  bool dropped_closing_waypoint_ = false;
  std::vector<Point> points_;
  std::vector<double> cumulative_s_;
  std::vector<double> segment_length_;
  std::vector<SplineSegment> spline_x_;
  std::vector<SplineSegment> spline_y_;
  double total_length_m_ = 0.0;
  ProjectionConfig projection_config_;

  bool track_widths_valid_ = false;
  double width_spacing_m_ = 0.0;
  std::vector<double> raw_right_m_;
  std::vector<double> raw_left_m_;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_REFERENCE_RACELINE_REFERENCE_HPP
