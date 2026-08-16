#ifndef LOCAL_PLANNING_REFERENCE_RACELINE_REFERENCE_HPP
#define LOCAL_PLANNING_REFERENCE_RACELINE_REFERENCE_HPP

#include "local_planning/core/types.hpp"

#include <cstddef>
#include <vector>

namespace local_planning
{

// Tuning for the locally-seeded projection.
struct ProjectionConfig
{
  // Half-width of the arc-length window searched either side of the seed.
  // Cost scales with this: every project() scans +/- this much raceline, and
  // staysOnSide()/maximumOffsetDeviation() call project() once per path sample.
  // Too small is not a graceful degradation -- once the true station falls
  // outside the window the search returns a confidently wrong answer without
  // setting seed_was_stale, so keep several times the worst seed step (ego
  // speed x cycle period, times however many cycles may be dropped).
  double seed_window_m = 2.0;
  // A candidate whose reference tangent disagrees with the query heading by
  // more than this is on the wrong branch.  The wrong branch of a hairpin is
  // typically anti-parallels
  double tangent_tolerance_rad = 1.2; //69 degrees
};

struct Projection
{
  double s = 0.0;
  double d = 0.0;   // signed lateral offset, positive left of the reference

  bool seed_was_stale = false;
};

struct SustainableBounds
{
  double right_magnitude = 0.0;
  double left_magnitude = 0.0;

  static SustainableBounds unbounded();
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

// The smooth closed reference the whole planner is anchored to.
// Geometry is a periodic cubic spline in x(s) and y(s) parameterized by
// cumulative chord length. Speed is interpolated linearly between waypoints
// Projection is locally seeded and orientation-checked.
// The seed is owned by the caller, not cached
class RacelineReference
{
public:
  // Returns false and leaves the object invalid if the loop is degenerate.
  bool setRacingLine(const std::vector<Point> & points);
  void setProjectionConfig(const ProjectionConfig & config) {projection_config_ = config;}
  const ProjectionConfig & projectionConfig() const {return projection_config_;}

  bool valid() const {return valid_;}
  double totalLength() const {return total_length_m_;}
  std::size_t waypointCount() const {return points_.size();}

  // Widths are index-aligned with the active reference. Construction interpolates
  // onto a uniform table; rawBounds(s) is an O(1) lookup, min'd with the next bin.
  bool setTrackWidths(
    const std::vector<TrackWidth> & widths,
    double spacing_m);
  void clearTrackWidths();
  bool trackWidthsValid() const {return track_widths_valid_;}
  SustainableBounds rawBounds(double s) const;
  WidthLookupSample widthSample(std::size_t index) const;
  std::size_t widthSampleCount() const {return raw_left_m_.size();}

  // Position, tangent, normal, heading, curvature, and speed at arc length s.
  // s is wrapped, so any real value is in range.
  ReferenceGeometrySample sampleAtS(double s) const;

  // Raceline speed only. wrapS + segmentAt + lerp, no spline eval.
  double velocityAtS(double s) const;

  // Frenet (s, d) -> world, offsetting along the left normal at s.
  Point toCartesian(double s, double d) const;

  // Signed lateral offset of p, positive left, refined from a caller-supplied
  // station.  Use this wherever the station is already known by construction --
  // every CurveSample carries raceline_s -- instead of paying project() to
  // rediscover it by scanning the seed window.  Measured ~50x cheaper.
  //
  // The hint only has to be near: Newton on arc length converges to the true
  // perpendicular foot from a hint off by a metre.  It is not a substitute for
  // project() when the station is genuinely unknown, and it will not cross to a
  // far branch of the track.  Pass converged to find out whether it got there;
  // when it comes back false the hint was not a neighbouring station and the
  // returned offset is meaningless.  When it converges, refined_s (if given) is
  // the station of the foot, to seed the next sample.
  double lateralOffsetAt(
    const Point & p, double s_hint, bool * converged = nullptr,
    double * refined_s = nullptr) const;

  // Locally-seeded projection with the tangent check.  Use this wherever the
  // query point has a meaningful heading
  Projection project(const Point & p, double heading, double seed_s) const;

  // Locally-seeded projection without the tangent check, for query points that
  // have no heading of their own, such as an occupied costmap cell.  The seed
  // window is still what keeps it on the right branch
  Projection project(const Point & p, double seed_s) const;

  // Unseeded fallback.  Correct only when nothing better is available.
  Projection projectGlobal(const Point & p, double heading, bool use_tangent_check) const;

  // Brings s into [0, total_length).
  double wrapS(double s) const;

  // Signed arc length from from_s to to_s, wrapped to [-L/2, L/2].  Positive
  // means to_s is ahead.  This is the only correct way to compare two s values.
  double deltaS(double from_s, double to_s) const;

private:
  struct SplineSegment
  {
    // Cubic in t = s - segment_start, valid over t in [0, length].
    double a = 0.0;   // value at t = 0
    double b = 0.0;   // first derivative at t = 0
    double c = 0.0;   // second derivative at t = 0, halved
    double d = 0.0;   // third derivative, sixthed
  };

  // Index of the segment containing wrapped arc length s, and t within it.
  std::size_t segmentAt(double s_wrapped, double & t) const;

  // Linear speed on an already-located segment. sampleAtS has i and t from
  // its own segmentAt; calling velocityAtS would search the raceline again.
  double velocityOnSegment(std::size_t i, double t) const;

  // Coarse-samples one segment, refines each guess, and keeps the result if it
  // beats best_dist_sq and passes the tangent check.  Returns true if it did.
  bool scanSegment(
    const Point & p,
    double heading,
    bool use_tangent_check,
    std::size_t segment,
    Projection & best,
    double & best_dist_sq) const;

  // Best projection on the forward arc starting at start_s for length_m.
  // Iterates only the overlapping segment index range.  found is false when
  // every candidate failed the tangent check.
  Projection searchArc(
    const Point & p,
    double heading,
    bool use_tangent_check,
    double start_s,
    double length_m,
    bool & found) const;

  // Same, over every segment.  No seed, because there is nothing to seed.
  Projection searchAllSegments(
    const Point & p,
    double heading,
    bool use_tangent_check,
    bool & found) const;

  // Newton refinement of the foot of the perpendicular within one segment.
  double refineOnSegment(const Point & p, std::size_t segment, double t_initial) const;

  bool valid_ = false;
  // True when setRacingLine() removed a repeated closing waypoint; the width
  // vector from the same message then legitimately has one extra entry.
  bool dropped_closing_waypoint_ = false;
  std::vector<Point> points_;
  std::vector<double> cumulative_s_;   // arc length at waypoint i
  std::vector<double> segment_length_; // segment i spans waypoint i to i+1 (wrapping)
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
