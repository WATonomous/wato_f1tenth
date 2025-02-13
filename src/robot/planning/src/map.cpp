#include "map.hpp"

Map::Map(std::string midline_path, std::string raceline_path) : m_size(0), r_size(0) {
    m_path = midline_path;
    r_path = raceline_path;

    if(generate_midline()){
        file_found = true;
        generate_angles_and_curvatures();
    }

    if(!generate_raceline()){
        file_found = false;
    }
    
}

bool Map::generate_raceline(){

    std::ifstream mapData(r_path);
    if (!mapData.is_open()) {
        return false;
    }

    std::string line;
    double x_m, y_m, theta, kappa, vel, accel, station;
    int file_size = 0;

    // Skip first three line (format comment)
    std::getline(mapData, line);
    std::getline(mapData, line);
    std::getline(mapData, line);

    while (std::getline(mapData, line)) {

        std::vector<double> data;
  
        for(size_t i = 0; i<line.length(); i++){
          if(line.at(i) == ';'){
            line.replace(i,1,1,' ');
          }
        }
  
        std::stringstream numStream(line);

        numStream >> station >> x_m >> y_m >> theta >> kappa >> vel >> accel;
  
        data.push_back(x_m);
        data.push_back(y_m);
        data.push_back(theta); 
        data.push_back(kappa);
        data.push_back(vel);
        data.push_back(accel);
        data.push_back(station);

        raceline.push_back(data);
        file_size++;
    }


    if(file_size > 1){
        r_size = file_size;
        return true;
    }

    mapData.close();
    return false;

}

bool Map::generate_midline(){

    std::ifstream mapData(m_path);
    if (!mapData.is_open()) {
        return false;
    }

    std::string line;
    double x_m, y_m, w_i, w_o;
    int file_size = 0;

    // Skip first line (format comment)
    std::getline(mapData, line);

    while (std::getline(mapData, line)) {

        std::vector<double> data;
  
        for(size_t i = 0; i<line.length(); i++){
          if(line.at(i) == ','){
            line.replace(i,1,1,' ');
          }
        }
  
        std::stringstream numStream(line);

        numStream >> x_m >> y_m >> w_i >> w_o;
  
        data.push_back(x_m);
        data.push_back(y_m);
        data.push_back(0); //theta to be updated
        data.push_back(0); //kappa to be updated
        data.push_back(w_i);
        data.push_back(w_o);

        midline.push_back(data);
        file_size++;
    }


    if(file_size > 1){
        m_size = file_size;
        return true;
    }

    mapData.close();
    return false;
}

void Map::generate_angles_and_curvatures(){

    for(int i = 0; i<m_size-1; i++){

        double angle, curvature, dx, dy, ds, dtheta;

        if(i == m_size-1){
            dx = midline[0][0] - midline[i][0];
            dy = midline[0][1] - midline[i][1];

            dtheta = midline[0][2] - midline[i][2]; // Angle difference        
        }
        else{
            dx = midline[i+1][0]- midline[i][0];
            dy = midline[i+1][1] - midline[i][1];

            dtheta = midline[i+1][2] - midline[i][2]; // Angle difference
        }

        angle = std::atan2(dy,dx);

        ds = std::sqrt(dx * dx + dy * dy);  // Step size (arc length difference), computed as a straight line as points are really close together
        curvature = (ds > 1e-6) ? (dtheta / ds) : 0.0;

        midline[i][2] = angle;
        midline[i][3] = curvature;
    }

}


int Map::get_midline_size(){
    return m_size;
}

int Map::get_raceline_size(){
    return r_size;
}

std::vector<double> Map::get_raceline(int idx){

    if (idx < 0 || idx >= r_size) {
        std::cerr << "Error: Index " << idx << " is out of bounds.\n";
        return {};
    }
    return raceline.at(idx);

}

std::vector<double> Map::get_midpoint(int idx){

    if (idx < 0 || idx >= m_size) {
        std::cerr << "Error: Index " << idx << " is out of bounds.\n";
        return {};
    }
    return midline.at(idx);

}

std::vector<double> Map::get_closest_raceline(std::vector<std::vector<double>> vertices){

    double min_dist = 1e9;
    double min_idx = -1;

    for(int i = 0; i < r_size; i++){
        
        for(const auto& vertex : vertices){
        
            double dist = std::sqrt(std::pow(raceline.at(i)[0] - vertex[0], 2) + std::pow(raceline.at(i)[1] - vertex[1], 2));

            if(dist < min_dist){
                min_dist = dist;
                min_idx = i;
            }

        }

    }

    return raceline.at(min_idx);

}