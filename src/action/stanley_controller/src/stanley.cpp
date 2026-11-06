#include "stanley.hpp"

#include <algorithm>
#include <cmath>

namespace stanley {

double wrap_angle(double a) {

    while (a >  M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;

}

void front_axle(const State& s, double wheelbase, double& fx, double& fy) {

    fx = s.x + wheelbase * std::cos(s.yaw);
    fy = s.y + wheelbase * std::sin(s.yaw);

}

void drop_duplicate_points(std::vector<geometry_msgs::msg::PoseStamped>& poses) {

    auto same_xy = [](const geometry_msgs::msg::PoseStamped& a,
                      const geometry_msgs::msg::PoseStamped& b) {
        return std::hypot(a.pose.position.x - b.pose.position.x,
                          a.pose.position.y - b.pose.position.y) < 1e-3;
    };

    poses.erase(std::unique(poses.begin(), poses.end(), same_xy), poses.end());
    while (poses.size() > 1 && same_xy(poses.front(), poses.back())) {
        poses.pop_back();
    }

}

Result compute(const State& s, const nav_msgs::msg::Path& path, size_t closest_idx, const Params& p) {

    const auto& poses = path.poses;
    const size_t n    = poses.size();

    double front_x, front_y;
    front_axle(s, p.wheelbase, front_x, front_y);

    //the path segment the front axle is on: closest -> next, or prev -> closest
    //when the axle hasn't reached the closest waypoint yet
    auto segment_from = [&](size_t i, double& ax, double& ay, double& tx, double& ty) {
        const auto& a = poses[i].pose.position;
        const auto& b = poses[(i + 1) % n].pose.position;
        ax = a.x; ay = a.y;
        tx = b.x - a.x; ty = b.y - a.y;
        double len = std::hypot(tx, ty);
        if (len > 1e-6) { tx /= len; ty /= len; }
    };
    double ax, ay, tx, ty;
    segment_from(closest_idx, ax, ay, tx, ty);

    //path heading at closest point (closest -> next). taken before the segment
    //below can step back: the previous segment's heading lags in tight turns
    //and the car runs wide
    double path_heading = std::atan2(ty, tx);

    if ((front_x - ax) * tx + (front_y - ay) * ty < 0.0) {
        segment_from((closest_idx + n - 1) % n, ax, ay, tx, ty);
    }

    Result r;

    //cross track error: signed distance from the front axle to the segment's
    //line, along its normal (+ = car is right of path, - = left). measuring to
    //the line instead of the waypoint keeps the along-track offset out of it,
    //which otherwise saw-tooths at the waypoint spacing whenever heading_error != 0
    r.cte = ty * (front_x - ax) - tx * (front_y - ay);

    r.heading_error = wrap_angle(path_heading - s.yaw);

    //stanley terms
    //k_soft keeps the atan2 from saturating at ~pi/2 when the car is nearly
    //stopped; abs() guards the denominator against rolling backward
    r.heading_term = p.k_h * r.heading_error;
    r.cte_term     = std::atan2(p.k_e * r.cte, std::abs(s.speed) + p.k_soft);

    r.steering = std::clamp(r.heading_term + r.cte_term, -p.max_steering_angle, p.max_steering_angle);

    //target speed from path z value: the slowest waypoint within speed_lookahead_s
    //of driving ahead, so the car starts braking before a slow corner, not in it
    r.speed = poses[closest_idx].pose.position.z;
    const double ahead = std::abs(s.speed) * p.speed_lookahead_s;
    double dist = 0.0;
    for (size_t k = 0, i = closest_idx; k + 1 < n && dist < ahead; k++) {
        const size_t j = (i + 1) % n;
        dist += std::hypot(poses[j].pose.position.x - poses[i].pose.position.x,
                           poses[j].pose.position.y - poses[i].pose.position.y);
        r.speed = std::min(r.speed, poses[j].pose.position.z);
        i = j;
    }
    if (p.speed_limit_active) {
        r.speed = std::clamp(r.speed, 0.0, p.speed_limit);
    }

    return r;

}

}  // namespace stanley
