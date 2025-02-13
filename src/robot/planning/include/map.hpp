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
    std::string path;
    int size;

    
    public:

    Map(std::string map_path);

    int get_size();
    
    bool generate_midline();
    void generate_angles_and_curvatures();

    bool file_found = false;

    std::vector<double> get_midpoint(int idx);
    

};

#endif