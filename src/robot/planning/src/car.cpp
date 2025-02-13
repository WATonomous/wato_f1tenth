#include "car.hpp"

Car::Car() : x(0), y(0), z(0), theta(0), q(0, 0, 0, 1), 
             forward_vector(1, 0, 0), 
             lateral_vector(0, 1, 0), 
             vertical_vector(0, 0, 1) {}

void Car::update_values(const geometry_msgs::msg::Pose car_pose){
    x = car_pose.position.x;
    y = car_pose.position.y;
    z = car_pose.position.z;

    q.setValue(car_pose.orientation.x, car_pose.orientation.y, car_pose.orientation.z, car_pose.orientation.w);

    update_unit_vectors(q);
    update_theta();
}

void Car::update_unit_vectors(const tf2::Quaternion& q){
    tf2::Matrix3x3 rotation_matrix(q);
  
    forward_vector = rotation_matrix.getColumn(0);
    lateral_vector = rotation_matrix.getColumn(1);
    vertical_vector = rotation_matrix.getColumn(2);
}


void Car::update_theta() {
    tf2::Vector3 x_unit(1, 0, 0);  // Reference X-axis vector

    if (forward_vector.length() > 0) {
        forward_vector.normalize();
    }
    double angle = std::atan2(forward_vector.y(), forward_vector.x());

    theta = angle;
}
