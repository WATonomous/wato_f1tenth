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
  // How far the nearest foot in the seed window may be before the window
  // itself, rather than the heading, is judged wrong.  This is what separates
  // "ego is sliding and its heading has stopped agreeing with the reference"
  // from "the seed has stopped tracking ego": the first leaves ego a plausible
  // lateral offset from the raceline, the second puts it arbitrarily far.  Keep
  // it above the widest real |d| (track half width plus overtake excursion plus
  // margin) and well below the scale at which a foot could belong to a
  // different part of the loop.
  double max_plausible_offset_m = 3.0;
};

struct Projection
{
  double s = 0.0;
  double d = 0.0;   // signed lateral offset, positive left of the reference

  // The seed window was abandoned and the whole loop searched.  Means ego_s is
  // not to be trusted until it recurs.
  bool seed_was_stale = false;
  // The answer came from the seed window, but only after the tangent tolerance
  // was widened past its configured value.  Expected during a slide or a hard
  // cut across the reference; a run of them outside those means
  // tangent_tolerance_rad is set tighter than the car actually drives.
  bool heading_check_relaxed = false;
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

  // Locally-seeded projection with the tangent check.  Use this wherever the
  // query point has a meaningful heading.
  //
  // Note there is no cheap station-hinted variant any more.  There used to be
  // one, for recovering the lateral offset of a path sample whose station was
  // roughly known; nothing needs it now that every CurveSample carries its
  // exact (raceline_s, d).  The only remaining query is the measured ego pose,
  // whose station genuinely is unknown, and that is what project() is for.
  Projection project(const Point & p, double heading, double seed_s) const;

  // Locally-seeded projection without the tangent check, for query points that
  // have no heading of their own, such as an occupied costmap cell.  The seed
  // window is still what keeps it on the right branch
  Projection project(const Point & p, double seed_s) const;

  // Unseeded fallback.  Correct only when nothing better is available.
  Projection projectGlobal(const Point & p, double heading, double tolerance_rad) const;

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
  // beats best_dist_sq and lands within tolerance_rad of the query heading.
  // Returns true if it did.  A tolerance of pi or more can never reject, which
  // is how callers with no meaningful heading opt out of the check.
  bool scanSegment(
    const Point & p,
    double heading,
    double tolerance_rad,
    std::size_t segment,
    Projection & best,
    double & best_dist_sq) const;

  // Best projection on the forward arc starting at start_s for length_m.
  // Iterates only the overlapping segment index range.  found is false when
  // every candidate failed the tangent check; best_dist_sq then keeps its
  // incoming value.
  Projection searchArc(
    const Point & p,
    double heading,
    double tolerance_rad,
    double start_s,
    double length_m,
    bool & found,
    double & best_dist_sq) const;

  // Same, over every segment.  No seed, because there is nothing to seed.
  Projection searchAllSegments(
    const Point & p,
    double heading,
    double tolerance_rad,
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
