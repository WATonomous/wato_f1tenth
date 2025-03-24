#ifndef ROTATIONS_H
#define ROTATIONS_H

#include <Eigen/Dense>

using namespace Eigen;

// Normalize angles to lie in range -pi < a[i] <= pi.
VectorXd angle_normalize(const VectorXd &a);

// Jacobian of RPY Euler angles with respect to axis-angle vector.
MatrixXd rpy_jacobian_axis_angle(const Vector3d &a);

// Skew symmetric form of a 3x1 vector.
Matrix3d skew_symmetric(const Vector3d &v);

// Quaternion class.
class Quaternion {
public:
    double w, x, y, z;

    Quaternion(double w_ = 1.0, double x_ = 0.0, double y_ = 0.0, double z_ = 0.0);
    Quaternion(const Vector3d &axis_angle);

    Matrix3d to_mat() const;
    Vector3d to_euler() const;
    Quaternion normalize() const;
    VectorXd to_numpy() const;
    Quaternion quat_mult_left(const Quaternion &q) const;
    Quaternion quat_mult_right(const Quaternion &q) const;
};

#endif // ROTATIONS_H
