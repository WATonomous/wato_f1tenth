#include "map.hpp"

Map::Map(std::string midline_path, std::string raceline_path) : m_size(0), r_size(0) {
    m_path = midline_path;
    r_path = raceline_path;

    if(!generate_raceline()){
        file_found = false;
    }

    if(generate_midline()){
        file_found = true;
        generate_angles_and_curvatures();
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

        for(size_t i = 0; i<line.length(); i++){
          if(line.at(i) == ';'){
            line.replace(i,1,1,' ');
          }
        }
  
        std::stringstream numStream(line);

        numStream >> station >> x_m >> y_m >> theta >> kappa >> vel >> accel;

        Point data = {x_m, y_m, normalise_angle(theta), kappa, vel, accel};

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
  
        for(size_t i = 0; i<line.length(); i++){
          if(line.at(i) == ','){
            line.replace(i,1,1,' ');
          }
        }
  
        std::stringstream numStream(line);

        numStream >> x_m >> y_m >> w_i >> w_o;
  
        //theta and kappa will be updated later
        Midpoint data = {x_m, y_m, 0,0, w_i, w_o};

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

        double angle, dx, dy;

        if(i == m_size-1){
            dx = midline[0].x - midline[i].x;
            dy = midline[0].y - midline[i].y;     
        }
        else{
            dx = midline[i+1].x- midline[i].x;
            dy = midline[i+1].y - midline[i].y;
        }

        angle = std::atan2(dy,dx);

        midline[i].theta = angle;
    }

    for(int i = 0; i<m_size-1; i++){

        double curvature, dx, dy, ds, dtheta;

        if(i == m_size-1){
            dx = midline[0].x - midline[i].x;
            dy = midline[0].y - midline[i].y;

            dtheta = midline[0].theta - midline[i].theta; // Angle difference        
        }
        else{
            dx = midline[i+1].x- midline[i].x;
            dy = midline[i+1].y - midline[i].y;

            dtheta = midline[i+1].theta - midline[i].theta; // Angle difference
        }

        ds = std::sqrt(dx * dx + dy * dy);  // Step size (arc length difference), computed as a straight line as points are really close together
        curvature = (ds > 1e-6) ? (dtheta / ds) : 0.0;

        midline[i].kappa = curvature;
    }

}


int Map::get_midline_size(){
    return m_size;
}

int Map::get_raceline_size(){
    return r_size;
}

Point Map::get_raceline(int idx){

    if (idx < 0 || idx >= r_size) {
        Point empty = {0,0,0,0};
        std::cerr << "Error: Index " << idx << " is out of bounds.\n";
        return empty;
    }
    return raceline.at(idx);

}

Midpoint Map::get_midpoint(int idx){

    if (idx < 0 || idx >= m_size) {
        std::cerr << "Error: Index " << idx << " is out of bounds.\n";
        return {};
    }
    return midline.at(idx);

}

Point Map::get_associated_raceline(std::vector<Point> vertices){

    double min_dist = 1e9;
    double min_idx = -1;

    for(int i = 0; i < r_size; i++){
        
        for(const auto& vertex : vertices){

            double dist = euc_dist(vertex.x, raceline.at(i).x, vertex.y, raceline.at(i).y);
        
            if(dist < min_dist){
                min_dist = dist;
                min_idx = i;
            }

        }

    }
    return raceline.at(min_idx);
}

Midpoint Map::get_closest_midline(Point pose, int offset){

    double min_dist = 1e9;
    double min_idx = -1;

    for(int i = 0; i < m_size; i++){
    
        double dist = euc_dist(pose.x,midline.at(i).x,pose.y,midline.at(i).y);

        if(dist < min_dist){
            min_dist = dist;
            min_idx = i;
        }
    }

    if(min_idx+offset > m_size){
        int idx = min_idx + offset - m_size;
        return midline.at(idx);
    }

    return midline.at(min_idx+offset);
}