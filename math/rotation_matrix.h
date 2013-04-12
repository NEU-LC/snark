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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#ifndef SNARK_GRAPHICS_IMPL_ROTATION_MATRIX_H_
#define SNARK_GRAPHICS_IMPL_ROTATION_MATRIX_H_

#include <comma/base/exception.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace snark {

/// rotation matrix (a quick and dirty class, to refactor at some point)
class rotation_matrix 
{
public:
    rotation_matrix( const ::Eigen::Matrix3d& rotation =  ::Eigen::Matrix3d::Identity() );
    rotation_matrix( const ::Eigen::Quaterniond& quaternion );
    rotation_matrix( const ::Eigen::Vector3d& rpy );

    const ::Eigen::Matrix3d& rotation() const;
    ::Eigen::Quaterniond quaternion() const;
    ::Eigen::Vector3d roll_pitch_yaw() const;
    static ::Eigen::Vector3d roll_pitch_yaw( const ::Eigen::Matrix3d& m );
    static ::Eigen::Matrix3d rotation( const ::Eigen::Vector3d& rpy );
    static ::Eigen::Matrix3d rotation( double roll, double pitch, double yaw );

    /// convert to requested type
    template< typename Output >
    Output convert() const;

private:
    ::Eigen::Matrix3d m_rotation;
    
};

} // namespace snark {

#endif // SNARK_GRAPHICS_IMPL_ROTATION_MATRIX_H_
