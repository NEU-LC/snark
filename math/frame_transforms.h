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

#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "roll_pitch_yaw.h"

namespace snark { namespace frame_transforms {

///Denavit-Hartenberg parameters for robotic link
struct dh_transform
{
    dh_transform() : d(0), theta(0), r(0), alpha(0) {}
    dh_transform(double d_, double theta_, double r_, double alpha_) : d(d_), theta(theta_), r(r_), alpha(alpha_) {}
    double d;
    double theta;
    double r;
    double alpha;
};

struct transform
{
    Eigen::Vector3d translation;
    snark::roll_pitch_yaw rotation;
    
    transform() : translation( Eigen::Vector3d::Zero() ), rotation( 0, 0, 0 ) {}
    
    ::Eigen::Affine3d affine() const;
};

struct tr_transform
{
    tr_transform() : translation(Eigen::Vector3d::Zero()), rotation(1,0,0,0){}
    Eigen::Vector3d translation;
    Eigen::Quaternion<double> rotation;
};

/// inverts a homogeneous transform using transpose formula
Eigen::Matrix4d inverse_transform(const Eigen::Matrix4d& T);

/// provides the homogeneous transform from rotation matrix and translation vector
Eigen::Matrix4d homogeneous_transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

/// converts homogeneous transform to tr
tr_transform matrix_to_tr(const Eigen::Matrix4d& T);

/// provides the homogeneous transform from the dh parameters
Eigen::Matrix4d dh_to_matrix(const dh_transform& T_dh);

/// dh to tr
tr_transform dh_to_tr(const dh_transform& T_dh);

}} // namespace snark { namespace frame_transforms {

#endif // TRANSFORMS_H
