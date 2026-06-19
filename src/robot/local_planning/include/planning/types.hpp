#ifndef PLANNING_TYPES_HPP
#define PLANNING_TYPES_HPP

#include <vector>
#include <cstdint>

namespace local_planning {

struct Point {
    double x;
    double y;
    double velocity;

    Point(double x = 0.0, double y = 0.0, double velocity = 0.0) : x(x), y(y), velocity(velocity) {}
};

struct FrenetPoint {
    double s;
    double d;
};

struct Odometry {
    Point position;
    double velocity;
    double heading;
};

struct OccupancyGrid {
    std::vector<int8_t> data; //  -1 (unknown), otherwise 0 to 100 for p(occupied)
    int width;
    int height;
    double resolution;
    Point origin;
};



} // namespace local_planning

#endif // PLANNING_TYPES_HPP
