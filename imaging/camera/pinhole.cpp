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

#include "pinhole.h"

namespace snark { namespace camera {

pinhole::distortion_t::operator Eigen::Matrix< double, 5, 1 >() const
{
    Eigen::Matrix< double, 5, 1 > m;
    m[0] = radial.k1;
    m[1] = radial.k2;
    m[2] = tangential.p1;
    m[3] = tangential.p2;
    m[4] = radial.k3;
    return m;
}

Eigen::Vector2d pinhole::pixel_size() const { return Eigen::Vector2d( sensor_size.x() / image_size.x(), sensor_size.y() / image_size.y() ); }

static double squared_radius( const Eigen::Vector2d& p, const snark::camera::pinhole& c )
{
    return ( p - ( c.principal_point ? *c.principal_point : c.image_centre() ) ).squaredNorm();
}

Eigen::Vector2d pinhole::radially_corrected( const Eigen::Vector2d& p ) const
{
    double r2 = squared_radius( p, *this );
    double k = 1 + distortion.radial.k1 * r2 + distortion.radial.k2 * r2 * r2 + distortion.radial.k3 * r2 * r2 * r2;
    return p * k;
}

Eigen::Vector2d pinhole::tangentially_corrected( const Eigen::Vector2d& p ) const
{
    double r2 = squared_radius( p, *this );
    double xy = p.x() * p.y();
    return p + Eigen::Vector2d( distortion.tangential.p1 * 2 * xy + distortion.tangential.p2 * ( r2 + p.x() * p.x() * 2 )
                              , distortion.tangential.p2 * 2 * xy + distortion.tangential.p1 * ( r2 + p.y() * p.y() * 2 ) );
}

Eigen::Vector2d pinhole::undistorted( const Eigen::Vector2d& p ) const { return tangentially_corrected( radially_corrected( p ) ); }

Eigen::Vector3d pinhole::to_cartesian( const Eigen::Vector2d& p, bool undistort ) const
{
    Eigen::Vector2d q = ( undistort ? undistorted( p ) : p ) - ( principal_point ? *principal_point : image_centre() );
    Eigen::Vector2d s = pixel_size();
    return Eigen::Vector3d( q.x() * s.x(), -q.y() * s.x(), -focal_length ); // todo: verify signs
}

Eigen::Vector2d pinhole::image_centre() const { return Eigen::Vector2d( double( image_size.x() ) / 2, double( image_size.y() ) / 2 ); }

} } // namespace snark { namespace camera {
