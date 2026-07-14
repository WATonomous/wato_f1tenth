#ifndef PLANNING_FRENET_CONVERTER_HPP
#define PLANNING_FRENET_CONVERTER_HPP

#include "planning/types.hpp"

#include <vector>

namespace local_planning
{

class FrenetConverter
{
public:
  FrenetConverter();

  void setRacingLine(const std::vector<Point> & racing_line);

  FrenetPoint cartesianToFrenet(const Point & p) const;
  Point frenetToCartesian(const FrenetPoint & fp) const;
  static Point frenetToCartesian(const ReferenceGeometrySample & ref, double d);
  /*
  the math for the below functions are pretty gnarly and its crucial to
  note that they are approximations
  TODO: make better approximations for them all??
  */
  ReferenceGeometrySample sampleAtS(double s) const;
  double getRacingLineHeading(double s) const;
  double getRacingLineVelocity(double s) const;
  double getRacingLineCurvature(double s) const;
  double getRacingLineCurvatureDerivative(double s) const;
  double getTotalLength() const;

  void fillReferenceGeometryTable(
    const std::vector<double> & s_values,
    std::vector<ReferenceGeometrySample> & out) const;

  // out[layer * sample_count + i]
  void fillUniformReferenceGeometryTable(
    double s_start,
    int layer_count,
    double layer_spacing_m,
    int sample_count,
    std::vector<ReferenceGeometrySample> & out) const;

private:
  int findClosestPoint(const Point & p) const;
  double wrapS(double s) const;

  std::vector<Point> racing_line_;
  std::vector<double> distance_prefix_sum_;
  std::vector<double> waypoint_curvatures_;
  double total_length_;
};

} // namespace local_planning

#endif // PLANNING_FRENET_CONVERTER_HPP
