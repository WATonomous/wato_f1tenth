#ifndef COMMON_H_
#define COMMON_H_

#define _USE_MATH_DEFINES
#include <cmath>


struct Point {
    double x, y, theta, kappa, vel, accel, time;

    // Default constructor (initializes everything to 0)
    Point() : x(0), y(0), theta(0), kappa(0), vel(0), accel(0), time(0) {}

    // Parameterized constructor with default values for optional parameters
    Point(double x, double y, double theta, double kappa, double vel = -1, double accel = -1, double time = -1)
        : x(x), y(y), theta(theta), kappa(kappa), vel(vel), accel(accel), time(time) {}
};

struct Midpoint{
    double x, y, theta, kappa, w_i, w_o;
};

inline double euc_dist(double x1,double x2,double y1,double y2){
    return std::sqrt(std::pow(y2 - y1, 2) + std::pow(x2 - x1, 2));
}

inline double normalise_angle(double theta){
    double normalised_theta = theta;
    if(theta > M_PI){
        normalised_theta = theta - 2.0 * M_PI;
    }
    else if(theta < -M_PI){
        normalised_theta = theta + 2.0 * M_PI;
    }

    return normalised_theta;
}

#endif