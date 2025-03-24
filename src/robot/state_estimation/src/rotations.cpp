#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;

// Normalize angles to lie in range -pi < a[i] <= pi.
VectorXd angle_normalize(const VectorXd &a) {
    VectorXd result = a;
    result = result.unaryExpr([](double angle) { return std::fmod(angle, 2 * M_PI); });
    for (int i = 0; i < result.size(); ++i) {
        if (result[i] <= -M_PI) result[i] += 2 * M_PI;
        if (result[i] > M_PI) result[i] -= 2 * M_PI;
    }
    return result;
}

// Skew symmetric form of a 3x1 vector.
Matrix3d skew_symmetric(const Vector3d &v) {
    Matrix3d skew;
    skew <<  0, -v(2),  v(1),
             v(2),  0, -v(0),
            -v(1), v(0),  0;
    return skew;
}

// Jacobian of RPY Euler angles with respect to axis-angle vector.
MatrixXd rpy_jacobian_axis_angle(const Vector3d &a) {
    if (a.size() != 3) {
        throw std::invalid_argument("'a' must be a Vector3d with length 3.");
    }

    double na = a.norm();
    double na3 = std::pow(na, 3);
    double t = na;
    Vector3d u = a / t;

    // First-order approximation of Jacobian wrt u, t.
    MatrixXd Jr(3, 4);
    Jr.setZero();
    Jr(0, 0) = t / (std::pow(t, 2) * std::pow(u(0), 2) + 1);
    Jr(0, 3) = u(0) / (std::pow(t, 2) * std::pow(u(0), 2) + 1);
    Jr(1, 1) = t / std::sqrt(1 - std::pow(t * u(1), 2));
    Jr(1, 3) = u(1) / std::sqrt(1 - std::pow(t * u(1), 2));
    Jr(2, 2) = t / (std::pow(t, 2) * std::pow(u(2), 2) + 1);
    Jr(2, 3) = u(2) / (std::pow(t, 2) * std::pow(u(2), 2) + 1);

    // Jacobian of u, t wrt a.
    MatrixXd Ja(4, 3);
    Ja <<  (std::pow(a(1), 2) + std::pow(a(2), 2)) / na3, -(a(0) * a(1)) / na3, -(a(0) * a(2)) / na3,
           -(a(0) * a(1)) / na3, (std::pow(a(0), 2) + std::pow(a(2), 2)) / na3, -(a(1) * a(2)) / na3,
           -(a(0) * a(2)) / na3, -(a(1) * a(2)) / na3, (std::pow(a(0), 2) + std::pow(a(1), 2)) / na3,
            a(0) / na, a(1) / na, a(2) / na;

    return Jr * Ja;
}

class Quaternion {
public:
    double w, x, y, z;

    Quaternion(double w_ = 1.0, double x_ = 0.0, double y_ = 0.0, double z_ = 0.0) :
        w(w_), x(x_), y(y_), z(z_) {}

    Quaternion(const Vector3d &axis_angle) {
        double norm = axis_angle.norm();
        w = std::cos(norm / 2);
        if (norm < 1e-50) {
            x = 0;
            y = 0;
            z = 0;
        } else {
            Vector3d imag = axis_angle / norm * std::sin(norm / 2);
            x = imag(0);
            y = imag(1);
            z = imag(2);
        }
    }

    Matrix3d to_mat() const {
        Vector3d v(x, y, z);
        return (w * w - v.dot(v)) * Matrix3d::Identity() +
               2 * v * v.transpose() +
               2 * w * skew_symmetric(v);
    }

    Vector3d to_euler() const {
        double roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        double pitch = std::asin(2 * (w * y - z * x));
        double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        return Vector3d(roll, pitch, yaw);
    }

    Quaternion normalize() const {
        double norm = std::sqrt(w * w + x * x + y * y + z * z);
        return Quaternion(w / norm, x / norm, y / norm, z / norm);
    }

     VectorXd to_numpy() const {
        VectorXd q(4);
        q << w, x, y, z;
        return q;
    }

    Quaternion quat_mult_left(const Quaternion &q) const {
        double nw = w * q.w - x * q.x - y * q.y - z * q.z;
        double nx = w * q.x + x * q.w + y * q.z - z * q.y;
        double ny = w * q.y - x * q.z + y * q.w + z * q.x;
        double nz = w * q.z + x * q.y - y * q.x + z * q.w;
        return Quaternion(nw, nx, ny, nz);
    }

    Quaternion quat_mult_right(const Quaternion &q) const {
        double nw = q.w * w - q.x * x - q.y * y - q.z * z;
        double nx = q.w * x + q.x * w - q.y * z + q.z * y;
        double ny = q.w * y + q.x * z + q.y * w - q.z * x;
        double nz = q.w * z - q.x * y + q.y * x + q.z * w;
        return Quaternion(nw, nx, ny, nz);
    }

};


//-----------------
#include "rotations.h"
#include <cmath>
#include <stdexcept>

using namespace Eigen;

VectorXd angle_normalize(const VectorXd &a) {
    VectorXd result = a;
    result = result.unaryExpr([](double angle) { return std::fmod(angle, 2 * M_PI); });
    for (int i = 0; i < result.size(); ++i) {
        if (result[i] <= -M_PI) result[i] += 2 * M_PI;
        if (result[i] > M_PI) result[i] -= 2 * M_PI;
    }
    return result;
}

MatrixXd rpy_jacobian_axis_angle(const Vector3d &a) {
    if (a.size() != 3) {
        throw std::invalid_argument("'a' must be a Vector3d with length 3.");
    }

    double na = a.norm();
    double na3 = std::pow(na, 3);
    double t = na;
    Vector3d u = a / t;

    MatrixXd Jr(3, 4);
    Jr.setZero();
    Jr(0, 0) = t / (std::pow(t, 2) * std::pow(u(0), 2) + 1);
    Jr(0, 3) = u(0) / (std::pow(t, 2) * std::pow(u(0), 2) + 1);
    Jr(1, 1) = t / std::sqrt(1 - std::pow(t * u(1), 2));
    Jr(1, 3) = u(1) / std::sqrt(1 - std::pow(t * u(1), 2));
    Jr(2, 2) = t / (std::pow(t, 2) * std::pow(u(2), 2) + 1);
    Jr(2, 3) = u(2) / (std::pow(t, 2) * std::pow(u(2), 2) + 1);

    MatrixXd Ja(4, 3);
    Ja << (std::pow(a(1), 2) + std::pow(a(2), 2)) / na3, -(a(0) * a(1)) / na3, -(a(0) * a(2)) / na3,
        -(a(0) * a(1)) / na3, (std::pow(a(0), 2) + std::pow(a(2), 2)) / na3, -(a(1) * a(2)) / na3,
        -(a(0) * a(2)) / na3, -(a(1) * a(2)) / na3, (std::pow(a(0), 2) + std::pow(a(1), 2)) / na3,
        a(0) / na, a(1) / na, a(2) / na;

    return Jr * Ja;
}

Matrix3d skew_symmetric(const Vector3d &v) {
    Matrix3d skew;
    skew << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return skew;
}

Quaternion::Quaternion(double w_, double x_, double y_, double z_)
    : w(w_), x(x_), y(y_), z(z_) {}

Quaternion::Quaternion(const Vector3d &axis_angle) {
    double norm = axis_angle.norm();
    w = std::cos(norm / 2);
    if (norm < 1e-50) {
        x = 0;
        y = 0;
        z = 0;
    } else {
        Vector3d imag = axis_angle / norm * std::sin(norm / 2);
        x = imag(0);
        y = imag(1);
        z = imag(2);
    }
}

Matrix3d Quaternion::to_mat() const {
    Vector3d v(x, y, z);
    return (w * w - v.dot(v)) * Matrix3d::Identity() +
           2 * v * v.transpose() +
           2 * w * skew_symmetric(v);
}

Vector3d Quaternion::to_euler() const {
    double roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    double pitch = std::asin(2 * (w * y - z * x));
    double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    return Vector3d(roll, pitch, yaw);
}

Quaternion Quaternion::normalize() const {
    double norm = std::sqrt(w * w + x * x + y * y + z * z);
    return Quaternion(w / norm, x / norm, y / norm, z / norm);
}

VectorXd Quaternion::to_numpy() const {
    VectorXd q(4);
    q << w, x, y, z;
    return q;
}

Quaternion Quaternion::quat_mult_left(const Quaternion &q) const {
    double nw = w * q.w - x * q.x - y * q.y - z * q.z;
    double nx = w * q.x + x * q.w + y * q.z - z * q.y;
    double ny = w * q.y - x * q.z + y * q.w + z * q.x;
    double nz = w * q.z + x * q.y - y * q.x + z * q.w;
    return Quaternion(nw, nx, ny, nz);
}

Quaternion Quaternion::quat_mult_right(const Quaternion &q) const {
    double nw = q.w * w - q.x * x - q.y * y - q.z * z;
    double nx = q.w * x + q.x * w - q.y * z + q.z * y;
    double ny = q.w * y + q.x * z + q.y * w - q.z * x;
    double nz = q.w * z - q.x * y + q.y * x + q.z * w;
    return Quaternion(nw, nx, ny, nz);
}


