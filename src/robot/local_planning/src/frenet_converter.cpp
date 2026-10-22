#include "planning/frenet_converter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_planning {

FrenetConverter::FrenetConverter() : total_length_(0.0) {}

void FrenetConverter::setRacingLine(const std::vector<Point>& racing_line) {
    racing_line_ = racing_line;

    //distance_prefix_sum_[i] = 
    //linear distance of (0 to 1) + (1 to 2) + ... + (n-1 to n)    
    distance_prefix_sum_.resize(racing_line_.size());
    distance_prefix_sum_[0] = 0.0;
    for (int i = 1; i < static_cast<int>(racing_line_.size()); i++) {
        //get the differential from i-1 to i
        double dx = racing_line_[i].x - racing_line_[i - 1].x; 
        double dy = racing_line_[i].y - racing_line_[i - 1].y;
        distance_prefix_sum_[i] = distance_prefix_sum_[i - 1] + std::hypot(dx, dy);
    }

    /*
    to get the total length we need to close the last part of the loop
    the distance from point n back to point 1
    */
    double dx = racing_line_.front().x - racing_line_.back().x;
    double dy = racing_line_.front().y - racing_line_.back().y;
    total_length_ = distance_prefix_sum_.back() + std::hypot(dx, dy);

    // precompute curvature at each waypoint (closed loop)
    // theta[i] = heading of segment i to (i+1)%n
    // kappa[i] = angle_diff(theta[i], theta[i-1]) / avg_seg_len
    int n = static_cast<int>(racing_line_.size());
    std::vector<double> seg_headings(n);
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        double sdx = racing_line_[j].x - racing_line_[i].x;
        double sdy = racing_line_[j].y - racing_line_[i].y;
        seg_headings[i] = std::atan2(sdy, sdx);
    }

    waypoint_curvatures_.resize(n);
    for (int i = 0; i < n; ++i) {
        int prev = (i - 1 + n) % n;

        double dtheta = seg_headings[i] - seg_headings[prev];
        dtheta = std::atan2(std::sin(dtheta), std::cos(dtheta)); // wrap to [-pi, pi]

        // length of the segment ending at waypoint i (prev to i)
        double len_prev;
        if (prev == n - 1) {
            len_prev = total_length_ - distance_prefix_sum_[prev];
        } else {
            len_prev = distance_prefix_sum_[prev + 1] - distance_prefix_sum_[prev];
        }

        // length of the segment starting at waypoint i (i ti next)
        double len_curr;
        if (i == n - 1) {
            len_curr = total_length_ - distance_prefix_sum_[i];
        } else {
            len_curr = distance_prefix_sum_[i + 1] - distance_prefix_sum_[i];
        }

        double avg_len = (len_prev + len_curr) * 0.5;
        if (avg_len > 1e-12) {
            waypoint_curvatures_[i] = dtheta / avg_len;
        } else {
            waypoint_curvatures_[i] = 0.0;
        }
    }
}

double FrenetConverter::getTotalLength() const {
    return total_length_;
}

double FrenetConverter::wrapS(double s) const {
    // bring s back into the interval [0, total length)
    s = std::fmod(s, total_length_);
    if (s < 0.0) {
        s += total_length_;
    }
    return s;
}

// Returns the index of the racing line waypoint closest to p.
// todo: use linear interpolation to improve accuracy

int FrenetConverter::findClosestPoint(const Point& p) const {
    int best_idx = 0;
    double best_dist = std::numeric_limits<double>::max();

    for (int i = 0; i < static_cast<int>(racing_line_.size()); i++) {
        double dist = std::hypot(p.x - racing_line_[i].x, p.y - racing_line_[i].y);
        if (dist < best_dist) {
            best_dist = dist;
            best_idx = i;
        }
    }

    return best_idx;
}

FrenetPoint FrenetConverter::cartesianToFrenet(const Point& p) const {
    int i = findClosestPoint(p);
    double s = distance_prefix_sum_[i];

    // d = signed lateral distance from waypoint i to p
    // heading direction is (cos θ, sin θ), so the left normal is (-sin θ, cos θ)
    double heading = getRacingLineHeading(s);
    double dx = p.x - racing_line_[i].x;
    double dy = p.y - racing_line_[i].y;
    double d = dx * (-std::sin(heading)) + dy * std::cos(heading);

    return {wrapS(s), d};
}

Point FrenetConverter::frenetToCartesian(const FrenetPoint& fp) const {
    int n = static_cast<int>(racing_line_.size());
    double s = wrapS(fp.s);

    // binary search to find the segment that contains s
    // distance_prefix_sum_[i] <= s < distance_prefix_sum_[i+1]
    int lo = 0;
    int hi = n - 1;
    while (lo < hi - 1) {
        int mid = (lo + hi) / 2;
        if (distance_prefix_sum_[mid] <= s) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    int i = lo;
    int j = (i + 1) % n;

    // how far along the segment ia s
    double seg_start = distance_prefix_sum_[i];
    double abx = racing_line_[j].x - racing_line_[i].x;
    double aby = racing_line_[j].y - racing_line_[i].y;
    double seg_len = std::hypot(abx, aby);

    double t = 0.0;
    double nx = 0.0;
    double ny = 0.0;
    if (seg_len > 1e-12) {
        t  = (s - seg_start) / seg_len;
        
        //left normal is (-sinx, cosx)
        nx = -aby / seg_len; 
        ny =  abx / seg_len;
    }

    // point on racing line at s which is offset laterally by d
    double cx = racing_line_[i].x + t * abx;
    double cy = racing_line_[i].y + t * aby;

    // target speed from the racing line at this arc length position
    double velocity = racing_line_[i].velocity + t * (racing_line_[j].velocity - racing_line_[i].velocity);

    return {cx + fp.d * nx, cy + fp.d * ny, velocity};
}

double FrenetConverter::getRacingLineHeading(double s) const {
    int n = static_cast<int>(racing_line_.size());
    s = wrapS(s);

    // binary search for segment
    int lo = 0;
    int hi = n - 1;
    while (lo < hi - 1) {
        int mid = (lo + hi) / 2;
        if (distance_prefix_sum_[mid] <= s) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    int i = lo;
    int j = (i + 1) % n;

    double dx = racing_line_[j].x - racing_line_[i].x;
    double dy = racing_line_[j].y - racing_line_[i].y;
    return std::atan2(dy, dx);
}

double FrenetConverter::getRacingLineVelocity(double s) const {
    int n = static_cast<int>(racing_line_.size());
    s = wrapS(s);

    // binary search for segment
    int lo = 0;
    int hi = n - 1;
    while (lo < hi - 1) {
        int mid = (lo + hi) / 2;
        if (distance_prefix_sum_[mid] <= s) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    int i = lo;
    int j = (i + 1) % n;

    return std::min(racing_line_[i].velocity, racing_line_[j].velocity);
}

double FrenetConverter::getRacingLineCurvature(double s) const {
    int n = static_cast<int>(racing_line_.size());
    s = wrapS(s);

    int lo = 0;
    int hi = n - 1;
    while (lo < hi - 1) {
        int mid = (lo + hi) / 2;
        if (distance_prefix_sum_[mid] <= s) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    int i = lo;
    int j = (i + 1) % n;

    double seg_len;
    if (j == 0) {
        seg_len = total_length_ - distance_prefix_sum_[i];
    } else {
        seg_len = distance_prefix_sum_[j] - distance_prefix_sum_[i];
    }

    double t = 0.0;
    if (seg_len > 1e-12) {
        t = (s - distance_prefix_sum_[i]) / seg_len;
    }
    return waypoint_curvatures_[i] + t * (waypoint_curvatures_[j] - waypoint_curvatures_[i]);
}

double FrenetConverter::getRacingLineCurvatureDerivative(double s) const {
    int n = static_cast<int>(racing_line_.size());
    s = wrapS(s);

    int lo = 0;
    int hi = n - 1;
    while (lo < hi - 1) {
        int mid = (lo + hi) / 2;
        if (distance_prefix_sum_[mid] <= s) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    int i = lo;
    int j = (i + 1) % n;

    double seg_len;
    if (j == 0) {
        seg_len = total_length_ - distance_prefix_sum_[i];
    } else {
        seg_len = distance_prefix_sum_[j] - distance_prefix_sum_[i];
    }

    if (seg_len < 1e-12) return 0.0;
    return (waypoint_curvatures_[j] - waypoint_curvatures_[i]) / seg_len;
}

} // namespace local_planning
