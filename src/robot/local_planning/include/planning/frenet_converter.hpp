#ifndef PLANNING_FRENET_CONVERTER_HPP
#define PLANNING_FRENET_CONVERTER_HPP

#include "types.hpp"

#include <vector>

namespace local_planning {

class FrenetConverter {
public:
    FrenetConverter();

    void setRacingLine(const std::vector<Point>& racing_line);

    FrenetPoint cartesianToFrenet(const Point& p) const;
    Point frenetToCartesian(const FrenetPoint& fp) const;

    double getRacingLineHeading(double s) const;
    double getRacingLineVelocity(double s) const;
    double getTotalLength() const;

private:
    int findClosestSegment(const Point& p) const;
    double wrapS(double s) const;

    std::vector<Point> racing_line_;
    std::vector<double> distance_prefix_sum_;
    double total_length_;
};

} // namespace local_planning

#endif // PLANNING_FRENET_CONVERTER_HPP
