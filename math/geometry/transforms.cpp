#include "transforms.h"

namespace snark{

Eigen::Matrix4d inverse_transform(Eigen::Matrix4d& T)
{
    return homogeneous_transform(T.topLeftCorner(3,3).transpose(),-T.topLeftCorner(3,3).transpose()*T.topRightCorner(3,1));
}

Eigen::Matrix4d homogeneous_transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
    Eigen::Matrix4d _T=Eigen::Matrix4d::Zero(); //temporary
    _T.topLeftCorner(3,3)=R;
    _T.topRightCorner(3,1)=t;
    _T(3,3)=1;
    return _T;
}

Eigen::Matrix4d dh_to_matrix(dh_transform T_dh)
{
    Eigen::Matrix4d T;
    T<<cos(T_dh.theta), -sin(T_dh.theta)*cos(T_dh.alpha), sin(T_dh.theta)*sin(T_dh.alpha), T_dh.r*cos(T_dh.theta), sin(T_dh.theta), cos(T_dh.theta)*cos(T_dh.alpha), -cos(T_dh.theta)*sin(T_dh.alpha), T_dh.r*sin(T_dh.theta), 0,  sin(T_dh.alpha), cos(T_dh.alpha), T_dh.d, 0, 0, 0, 1;
    return T;
}

tr_transform dh_to_tr(dh_transform T_dh)
{
    tr_transform T;
    double alpha=T_dh.alpha;
    double d=T_dh.d;
    double r=T_dh.r;
    double theta=T_dh.theta;
    T.translation<<r*cos(theta),r*sin(theta),d;
    T.rotation.w()=cos(alpha/2)*cos(theta/2);
    T.rotation.x()=sin(alpha/2)*cos(theta/2);
    T.rotation.y()=sin(alpha/2)*sin(theta/2);
    T.rotation.z()=cos(alpha/2)*sin(theta/2);
    return T;
}

}
