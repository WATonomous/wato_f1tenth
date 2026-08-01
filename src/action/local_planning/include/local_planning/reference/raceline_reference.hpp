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
  double seed_window_m = 3.0;
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

  bool valid() const {return valid_;}
  double totalLength() const {return total_length_m_;}
  std::size_t waypointCount() const {return points_.size();}

  // Position, tangent, normal, heading, curvature, and speed at arc length s.
  // s is wrapped, so any real value is in range.
  ReferenceGeometrySample sampleAtS(double s) const;

  // Frenet (s, d) -> world, offsetting along the left normal at s.
  Point toCartesian(double s, double d) const;

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

  // Coarse-samples one segment, refines each guess, and keeps the result if it
  // beats best_dist_sq and passes the tangent check.  Returns true if it did.
  bool scanSegment(
    const Point & p,
    double heading,
    bool use_tangent_check,
    std::size_t segment,
    Projection & best,
    double & best_dist_sq) const;

  // Best projection within +/- window of seed_s.  found is false when every
  // candidate failed the tangent check.
  Projection searchWindow(
    const Point & p,
    double heading,
    bool use_tangent_check,
    double seed_s,
    double window_m,
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
  std::vector<Point> points_;
  std::vector<double> cumulative_s_;   // arc length at waypoint i
  std::vector<double> segment_length_; // segment i spans waypoint i to i+1 (wrapping)
  std::vector<SplineSegment> spline_x_;
  std::vector<SplineSegment> spline_y_;
  double total_length_m_ = 0.0;
  ProjectionConfig projection_config_;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_REFERENCE_RACELINE_REFERENCE_HPP
