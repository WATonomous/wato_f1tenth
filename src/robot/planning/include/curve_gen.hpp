#ifndef CURVE_GEN_HPP_
#define CURVE_GEN_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>
#include "map.hpp"
#include "common.hpp"
#include "car.hpp"

class Lattice{

    std::vector<std::vector<Point>> vertice_group;
    std::vector<Point> raceline_vertices;
    std::vector<int> raceline_vert_idx;
  
    void generate_vertices(double DIST_PER_STEP ,int VERT_PER_STEP, double BUFFER, int& SAMPLE_SIZE);
    int step;
    

    Map map;

    //curve gen
    int resolution;

    Eigen::Matrix4d computeJacobian(const Point& start, const Point& target, Eigen::Vector3d params, int steps, double h = 1e-5);
    Eigen::Vector4d computeError(const Point& actual, const Point& target);
    Point generateSpiral(Point start, double a, double b, double c, double d, double sf, int steps, std::vector<Point>& points);
    void calculateSpiralCoeffs(double p0, double p1, double p2, double p3, double sf,double& a, double& b, double& c, double& d);

    int prev_curve_idx = -1;

    public:

    std::vector<Point> get_vertice_set(int idx) {return vertice_group.at(idx);}   
    Point get_raceline_vertex(int idx) { return raceline_vertices.at(idx); }
    Lattice(Map in_map, double DIST_PER_STEP ,int VERT_PER_STEP, int RES_PER_STEP, double BUFFER, int& SAMPLE_SIZE);
    Lattice();
    std::vector<std::vector<Point>> getTrajectories(Car ego_car,bool on_raceline = true, int current_idx=-1);
    void generateCurve(Point start, Point target, int steps, std::vector<Point>& points);
    int find_closest_vertices_idx(Car ego_car);
 
};

void generateCurve(Point start, Point target, int steps, std::vector<Point>& points);

#endif