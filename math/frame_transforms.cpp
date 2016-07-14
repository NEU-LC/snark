// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "frame_transforms.h"
#include "rotation_matrix.h"

namespace snark { namespace frame_transforms {

::Eigen::Affine3d transform::operator()() const
{
    ::Eigen::Translation3d t;
    t.vector() = translation;
    return ::Eigen::Affine3d( t * snark::rotation_matrix::rotation( rotation ) );
}
    
Eigen::Matrix4d inverse_transform(const Eigen::Matrix4d& T)
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

tr_transform matrix_to_tr(const Eigen::Matrix4d& T)
{
    tr_transform t;
    t.translation=T.topRightCorner(3,1);
    Eigen::Matrix3d rotation_matrix=T.topLeftCorner(3,3);
    t.rotation=Eigen::Quaterniond(rotation_matrix);
    return t;
}

Eigen::Matrix4d dh_to_matrix(const dh_transform& T_dh)
{
    Eigen::Matrix4d T;
    T<<cos(T_dh.theta), -sin(T_dh.theta)*cos(T_dh.alpha), sin(T_dh.theta)*sin(T_dh.alpha), T_dh.r*cos(T_dh.theta), sin(T_dh.theta), cos(T_dh.theta)*cos(T_dh.alpha), -cos(T_dh.theta)*sin(T_dh.alpha), T_dh.r*sin(T_dh.theta), 0,  sin(T_dh.alpha), cos(T_dh.alpha), T_dh.d, 0, 0, 0, 1;
    return T;
}

tr_transform dh_to_tr(const dh_transform& T_dh)
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

} } // namespace snark { namespace frame_transforms {
