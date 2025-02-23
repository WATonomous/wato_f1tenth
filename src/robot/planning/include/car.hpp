#ifndef CAR_HPP_
#define CAR_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <geometry_msgs/msg/pose.hpp>
#include "common.hpp"

class Car{

    //cars position
    double x,y,z;

    //cars orientation
    double theta;
    tf2::Quaternion q;

    //unit vectors
    tf2::Vector3 forward_vector;
    tf2::Vector3 lateral_vector;
    tf2::Vector3 vertical_vector;

    void update_unit_vectors(const tf2::Quaternion& q);
    void update_theta();

    public:
        Car();

        void update_values(const geometry_msgs::msg::Pose car_pose);

        double get_x() { return x;}
        double get_y() { return y;}
        double get_z() { return z;}
        
        tf2::Quaternion get_q() { return q;};
        double get_theta() { return theta;}

        tf2::Vector3 get_fv() { return forward_vector;}
        tf2::Vector3 get_lv() { return lateral_vector;}
        tf2::Vector3 get_vv() { return vertical_vector;}

};

#endif