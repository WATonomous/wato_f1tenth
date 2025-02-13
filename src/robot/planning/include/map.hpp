#ifndef MAP_HPP_
#define MAP_HPP_

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

class Map{

    std::vector<std::vector<double>> midline;
    std::vector<std::vector<double>> raceline;
    std::string m_path;
    std::string r_path;
    int m_size;
    int r_size;

    
    public:

    Map(std::string midline_path, std::string raceline_path);

    int get_midline_size();
    int get_raceline_size();
    
    bool generate_midline();
    void generate_angles_and_curvatures();

    bool generate_raceline();

    bool file_found = false;

    std::vector<double> get_midpoint(int idx);
    std::vector<double> get_raceline(int idx); 
    std::vector<double> get_closest_raceline(std::vector<std::vector<double>> vertices);

};

#endif