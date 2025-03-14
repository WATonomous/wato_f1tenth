#include "curve_gen.hpp"

Lattice::Lattice(Map in_map, double DIST_PER_STEP ,int VERT_PER_STEP, int RES_PER_STEP, double BUFFER, int& SAMPLE_SIZE){

    map = in_map;
    resolution = RES_PER_STEP;

    generate_vertices(DIST_PER_STEP, VERT_PER_STEP, BUFFER, SAMPLE_SIZE);

}

int Lattice::find_closest_vertices_idx(Car ego_car){

    Point pose = Point(ego_car.get_x(), ego_car.get_y(), ego_car.get_theta(), 0.0);
    Midpoint base = map.get_closest_midline(pose, step);

    double dist;
    double min_dist = 1e16;
    int min_idx = -1;

    for (size_t i = 0; i < vertice_group.size(); i++) {
        Point vertex = vertice_group[i][2]; //get middle point

        dist = euc_dist(base.x,vertex.x,base.y,vertex.y);
        if(dist < min_dist){
            min_dist = dist;
            min_idx = i;
        }
    }

    if(min_dist>-1){

        //check if point is in right direction
        double rejection_range = 0.9; //compared to pi
        double delta_angle = abs(std::atan2( (vertice_group[min_idx][2].y - base.y), (vertice_group[min_idx][2].x - base.x) )) - base.theta;
        if(delta_angle > rejection_range*M_PI && delta_angle < M_PI/rejection_range){
            if(min_idx + 1 < vertice_group.size()){
                return min_idx+1;
            }
            else{
                return 0;
            }
        }

        return min_idx;
    }
    else{
        std::cerr << "closest vertice group could not be found" << std::endl;
        return -1;
    }

}

std::vector<std::vector<Point>> Lattice::getTrajectories(Car ego_car, bool on_raceline, int current_idx){

    std::vector<std::vector<Point>> traj;
    Point start;
    std::vector<Point> target_set;

    int starting_idx = find_closest_vertices_idx(ego_car);

    if(starting_idx != prev_curve_idx){

        start = on_raceline ? raceline_vertices[starting_idx] : vertice_group[starting_idx][current_idx];

        if(starting_idx+1 == vertice_group.size()){
            target_set = vertice_group[0];
        }
        else{
            target_set = vertice_group[starting_idx+1];
        }

        for(const auto& target : target_set){
            std::vector<Point> points;
            generateCurve(start, target, resolution, points);
            traj.push_back(points);
        }

        if(on_raceline){
            std::vector<Point> points;

            int end_idx = starting_idx+1 == vertice_group.size() ? 0 : starting_idx+1;

            for(int i = raceline_vert_idx.at(starting_idx); i < raceline_vert_idx.at(end_idx); i++){
                points.push_back(map.get_raceline(i));
            }

            traj.push_back(points);

        }

        prev_curve_idx = starting_idx;
    }
    return traj;
}

void Lattice::generate_vertices(double DIST_PER_STEP ,int VERT_PER_STEP, double BUFFER, int& SAMPLE_SIZE)
{
  
  double VERTEX_X_OFFSET;

  int midline_size = map.get_midline_size();

  SAMPLE_SIZE = 0;
 
  for(int i = 0; i < midline_size; i++){

    static double local_dist = 0;
    static Midpoint curr_point = map.get_midpoint(0);
    static Midpoint next_point = map.get_midpoint(1);

    if(i != 0){
        curr_point = next_point;
        next_point = i+1 < midline_size ? map.get_midpoint(i+1) : map.get_midpoint(0);
    }

    if(local_dist < DIST_PER_STEP){
        local_dist += euc_dist(curr_point.x, next_point.x, curr_point.y, next_point.y);
    }
    else{
        std::vector<Point> vertices;

        for(int j = 0; j < VERT_PER_STEP; j++){

            if(j <= VERT_PER_STEP/2){
              VERTEX_X_OFFSET = (curr_point.w_i - BUFFER) / (VERT_PER_STEP/2);
            }
            else{
              VERTEX_X_OFFSET = (curr_point.w_o - BUFFER) / (VERT_PER_STEP/2);
            }
      
            double x = curr_point.x + (VERTEX_X_OFFSET * (j - VERT_PER_STEP/2) * std::cos(curr_point.theta + M_PI_2));
            double y = curr_point.y + (VERTEX_X_OFFSET * (j - VERT_PER_STEP/2) * std::sin(curr_point.theta + M_PI_2));
            double theta = curr_point.theta;
            double kappa = (curr_point.kappa > 1e-6) ? (1 / ((1 / curr_point.kappa) + VERTEX_X_OFFSET)) : 0.0;
      
            Point vertex = {x,y,theta,kappa};
            vertices.push_back(vertex);
        }

        local_dist = 0;
        SAMPLE_SIZE++;

        int raceline_idx;

        raceline_vertices.push_back(map.get_associated_raceline(vertices, raceline_idx));
        raceline_vert_idx.push_back(raceline_idx);
        vertice_group.push_back(vertices);
    }

  }

}

// Calculate cubic spiral coefficients
void Lattice::calculateSpiralCoeffs(double p0, double p1, double p2, double p3, double sf,
                           double& a, double& b, double& c, double& d) {
    a = p0;
    b = (-11 * p0 + 18 * p1 - 9 * p2 + 2 * p3) / (2 * sf);
    c = (9 * (2 * p0 - 5 * p1 + 4 * p2 - p3)) / (2 * sf * sf);
    d = (-9 * (p0 - 3 * p1 + 3 * p2 - p3)) / (2 * sf * sf * sf);
}


// Generate spiral and return final state
Point Lattice::generateSpiral(Point start, double a, double b, double c, double d, double sf, int steps, std::vector<Point>& points) {
    double ds = sf / steps;
    double x = start.x, y = start.y, theta = start.theta, accel = start.accel;
    double vel, time;

    for (int i = 1; i <= steps; ++i) {
        double s = i * ds;
        double kappa = a + b * s + c * s * s + d * s * s * s;
        theta += kappa * ds;
        x += cos(theta) * ds;
        y += sin(theta) * ds;
        vel = (start.vel + 2*accel*s) >= 0 ? std::sqrt(start.vel + 2*accel*s) : -1;
        time = accel == 0 ? sf/vel : (vel - start.vel)/accel;

        Point point = {x,y,theta,kappa,vel,accel,time};
        points.push_back(point);
    }

    return {x, y, theta, d};
}

// Compute the error between actual and target endpoints
Eigen::Vector4d Lattice::computeError(const Point& actual, const Point& target) {
    Eigen::Vector4d error;
    error << actual.x - target.x,
             actual.y - target.y,
             actual.theta - target.theta,
             actual.kappa - target.kappa;
    return error;
}

// Compute the Jacobian using finite differences
Eigen::Matrix4d Lattice::computeJacobian(const Point& start, const Point& target, Eigen::Vector3d params, int steps, double h) {
    Eigen::Matrix4d J;

    std::vector<Point> final_plus_points;
    std::vector<Point> final_minus_points;

    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3d params_plus = params;
        Eigen::Vector3d params_minus = params;

        params_plus[i] += h;
        params_minus[i] -= h;

        double a, b, c, d;
        calculateSpiralCoeffs(start.kappa, params_plus[0], params_plus[1], target.kappa, params_plus[2], a, b, c, d);
        Point final_plus = generateSpiral(start, a, b, c, d, params_plus[2], steps, final_plus_points);

        calculateSpiralCoeffs(start.kappa, params_minus[0], params_minus[1], target.kappa, params_minus[2], a, b, c, d);
        Point final_minus = generateSpiral(start, a, b, c, d, params_minus[2], steps, final_minus_points);

        Eigen::Vector4d error_plus = computeError(final_plus, target);
        Eigen::Vector4d error_minus = computeError(final_minus, target);

        Eigen::Vector4d derivative = (error_plus - error_minus) / (2 * h);
        J.col(i) = derivative;
    }
    return J;
}

// Iteratively refine curve using Newton's method
void Lattice::generateCurve(Point start, Point target, int steps, std::vector<Point>& points) {
    double p1 = (start.kappa + target.kappa)/2;
    double p2 = p1;
    double s = euc_dist(start.x, target.x, start.y, target.y);

    Eigen::Vector3d params(p1, p2, s);
    
    const int maxIterations = 20;
    const double tolerance = 0.25; //cm
    double norm;

    for (int iter = 0; iter < maxIterations; ++iter) {
        points.clear();

        double a, b, c, d;
        calculateSpiralCoeffs(start.kappa, params[0], params[1], target.kappa, params[2], a, b, c, d);
        Point final_state = generateSpiral(start, a, b, c, d, params[2], steps, points);

        Eigen::Vector4d error = computeError(final_state, target);
        norm = error.norm();

        std::cout << "Iteration " << iter << " | Error norm: " << norm << std::endl;

        if (norm < tolerance) break;

        Eigen::Matrix4d J = computeJacobian(start, target, params, steps);
        Eigen::Vector3d delta = J.topLeftCorner(3, 3).inverse() * error.head(3);

        params -= delta;
    }
    
    if (norm > tolerance){ 
        //if the paths are not converging, try repeat the process with Damping Factor (Levenberg–Marquardt Style)
        std::cerr << "Newton's method did not converge after 20 iterations. Final error: " << norm << "\n";
    }

}

// int main() {
//     Point start = {0.0, 0.0, 0.0, 0.0};
//     Point target = {10.0, 2.0, 0.5, 0.1};

//     Eigen::Vector3d params(0.05, 0.05, 10.0); // Initial guess: p1, p2, s_f

//     int steps = 100;
//     refineCurve(start, target, params, steps);

//     std::cout << "\nFinal Parameters: p1 = " << params[0] << ", p2 = " << params[1] << ", s_f = " << params[2] << std::endl;
//     return 0;
// }
