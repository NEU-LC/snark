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


#ifndef SNARK_ROTATION_MATRIX_H_
#define SNARK_ROTATION_MATRIX_H_

#include <Eigen/Geometry>
#include "snark/math/angle.h"

namespace snark {

class rotation_matrix 
{
public:
    rotation_matrix( const Eigen::Matrix3d& rotation =  Eigen::Matrix3d::Identity() ) : m_rotation( rotation ) {}
    rotation_matrix( const Eigen::Quaterniond& quaternion ) : m_rotation( quaternion.normalized() ) {}
    rotation_matrix( double roll, double pitch, double yaw )  : m_rotation( rotation( roll, pitch, yaw ) ) {}
    rotation_matrix( const Eigen::Vector3d& rpy ) : m_rotation( rotation( rpy.x(), rpy.y(), rpy.z() ) ) {}
    rotation_matrix( const Eigen::AngleAxisd& angle_axis ) : m_rotation( angle_axis ) {}
    
    const Eigen::Matrix3d& rotation() const { return m_rotation; }
    Eigen::Quaterniond quaternion() const { return Eigen::Quaterniond( m_rotation ); }
    Eigen::Vector3d roll_pitch_yaw() const { return roll_pitch_yaw( m_rotation ); }
    Eigen::Vector3d angle_axis() const
    {
        Eigen::AngleAxisd a( m_rotation );
        double angle = snark::math::angle< double >( snark::math::radians( a.angle() ) ).as_radians();
        Eigen::Vector3d axis = a.axis();
        if( comma::math::less( M_PI, angle ) ) { angle = 2*M_PI - angle; axis = -axis; }
        return axis * angle;    
    }
    
    static Eigen::Vector3d roll_pitch_yaw( const ::Eigen::Matrix3d& m )
    {
        double roll;
        double pitch=std::asin( -m(2,0) );
        double yaw;
        if( m(2,0)==1 || m(2,0)==-1 )
        {
            roll=0;
            yaw=std::atan2( m(1,2), m(0,2) );
        }
        else
        {
            roll=std::atan2( m(2,1), m(2,2) );
            yaw=std::atan2( m(1,0), m(0,0) );
        }
        Eigen::Vector3d rpy;
        rpy << roll, pitch, yaw;
        return rpy;
    }
    static Eigen::Matrix3d rotation( double roll, double pitch, double yaw )
    {
        const double sr = std::sin( roll );
        const double cr = std::cos( roll );
        const double sp = std::sin( pitch );
        const double cp = std::cos( pitch );
        const double sy = std::sin( yaw );
        const double cy = std::cos( yaw );
        const double spcy = sp*cy;
        const double spsy = sp*sy;
        Eigen::Matrix3d m;
        m << cp*cy, -cr*sy+sr*spcy,  sr*sy+cr*spcy,
             cp*sy,  cr*cy+sr*spsy, -sr*cy+cr*spsy,
             -sp,          sr*cp,          cr*cp;
        return m;
    }
    static Eigen::Matrix3d rotation( const ::Eigen::Vector3d& rpy ) { return rotation( rpy.x(), rpy.y(), rpy.z() ); }

private:
    Eigen::Matrix3d m_rotation;    
};

} // namespace snark {

#endif // SNARK_ROTATION_MATRIX_H_
