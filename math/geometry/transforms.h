#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace snark{

///D-H transform for robotic link
struct dh_transform
{
    dh_transform() : d(0), theta(0), r(0), alpha(0) {}
    dh_transform(double d_, double theta_, double r_, double alpha_) : d(d_), theta(theta_), r(r_), alpha(alpha_) {}
    double d;
    double theta;
    double r;
    double alpha;
};

struct tr_transform
{
    Eigen::Vector3d translation;
    Eigen::Quaternion<double> rotation;
};

/// inverts a homogeneous transform using transpose formula
Eigen::Matrix4d inverse_transform(Eigen::Matrix4d& T);

/// provides the homogeneous transform from rotation matrix and translation vector
Eigen::Matrix4d homogeneous_transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

/// provides the homogeneous transform from the dh parameters
Eigen::Matrix4d dh_to_matrix(dh_transform T_dh);

/// dh to tr
tr_transform dh_to_tr(dh_transform T_dh);

}

#endif // TRANSFORMS_H
