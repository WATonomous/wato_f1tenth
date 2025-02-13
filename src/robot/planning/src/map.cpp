#include "map.hpp"

Map::Map(std::string map_path) : size(0) {
    path = map_path;

    if(generate_midline()){
        file_found = true;
        generate_angles_and_curvatures();
    }
    
}

bool Map::generate_midline(){

    std::ifstream mapData(path);
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
        data.push_back(0);
        data.push_back(0);


        midline.push_back(data);
        file_size++;
    }


    if(file_size > 1){
        size = file_size;
        return true;
    }

    mapData.close();
    return false;
}

void Map::generate_angles_and_curvatures(){

    for(int i = 0; i<size-1; i++){

        double angle, curvature, dx, dy, ds, dtheta;

        if(i == size-1){
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


int Map::get_size(){
    return size;
}

std::vector<double> Map::get_midpoint(int idx){

    if (idx < 0 || idx >= size) {
        std::cerr << "Error: Index " << idx << " is out of bounds.\n";
        return {};
    }
    return midline.at(idx);

}