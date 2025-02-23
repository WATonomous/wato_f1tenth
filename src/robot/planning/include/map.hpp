#ifndef MAP_HPP_
#define MAP_HPP_

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

#include "common.hpp"

class Map{

    std::vector<Midpoint> midline;
    std::vector<Point> raceline;
    std::string m_path;
    std::string r_path;
    int m_size;
    int r_size;

    
    public:

    Map(std::string midline_path, std::string raceline_path);
    Map(){}

    int get_midline_size();
    int get_raceline_size();
    
    bool generate_midline();
    void generate_angles_and_curvatures();

    bool generate_raceline();

    bool file_found = false;

    Midpoint get_midpoint(int idx);
    Point get_raceline(int idx); 

    Point get_associated_raceline(std::vector<Point> vertices);
    Midpoint get_closest_midline(Point pose, int offset = 0);

};

#endif