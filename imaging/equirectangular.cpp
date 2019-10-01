// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2019 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

// snark is a generic and flexible library for robotics research
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

/// @author vsevolod vlaskine

#include <cmath>
#include <comma/base/exception.h>
#include "equirectangular.h"

namespace snark { namespace equirectangular {

cube::faces::values face_of( const Eigen::Vector3d& v ) // todo: watch performance, don't need to call it so many times
{
    double ax = std::abs( v.x() );
    double ay = std::abs( v.y() );
    double az = std::abs( v.z() );
    if( ax > ay && ax > az ) { return v.x() > 0 ? cube::faces::front : cube::faces::back; }
    if( ay > az ) { return v.y() > 0 ? cube::faces::right : cube::faces::left; }
    return v.z() > 0 ? cube::faces::bottom : cube::faces::top;
}

Eigen::Vector2d normalized( const Eigen::Vector2d& p, double spherical_width ) { return Eigen::Vector2d( p.x(), p.y() * 2 ) / spherical_width; }

std::pair< Eigen::Vector2d, cube::faces::values > to_cube( const Eigen::Vector2d& p, double spherical_width ) { return to_cube( normalized( p, spherical_width ) ); }

snark::range_bearing_elevation to_polar( const Eigen::Vector2d& p, double spherical_width ) { return to_polar( normalized( p, spherical_width ) ); }

snark::range_bearing_elevation to_polar( const Eigen::Vector2d& p ) { return snark::range_bearing_elevation( 0.5, M_PI * 2 * ( p.x() - 0.5 ), M_PI * ( p.y() - 0.5 ) ); }

Eigen::Vector3d to_cartesian( const Eigen::Vector2d& p )
{
    auto v = to_polar( p ).to_cartesian();
    double ax = std::abs( v.x() );
    double ay = std::abs( v.y() );
    double az = std::abs( v.z() );
    return v * 0.5 / std::max( ax, std::max( ay, az ) );
}

std::pair< Eigen::Vector2d, cube::faces::values > to_cube( const Eigen::Vector2d& p )
{
    auto c = to_polar( p ).to_cartesian();
    std::pair< Eigen::Vector2d, cube::faces::values > pixel;
    pixel.second = face_of( c );
    switch( pixel.second )
    {
        case cube::faces::top: pixel.first = Eigen::Vector2d( c.y(), c.x() ) / std::abs( c.z() ); break;
        case cube::faces::left: pixel.first = Eigen::Vector2d( c.x(), c.z() ) / std::abs( c.y() ); break;
        case cube::faces::front: pixel.first = Eigen::Vector2d( c.y(), c.z() ) / std::abs( c.x() ); break;
        case cube::faces::right: pixel.first = Eigen::Vector2d( -c.x(), c.z() ) / std::abs( c.y() ); break;
        case cube::faces::back: pixel.first = Eigen::Vector2d( -c.y(), c.z() ) / std::abs( c.x() ); break;
        case cube::faces::bottom: pixel.first = Eigen::Vector2d( c.y(), -c.x() ) / std::abs( c.z() ); break;
    }
    pixel.first *= 0.5;
    pixel.first.x() += 0.5;
    pixel.first.y() += 0.5;
    return pixel;
}

Eigen::Vector2d from_cube( const Eigen::Vector2d& p, cube::faces::values face )
{
    snark::range_bearing_elevation polar( cube::to_cartesian( p, face ) );
    return Eigen::Vector2d( polar.bearing() / ( M_PI * 2 ) + 0.5, ( polar.elevation() / M_PI + 0.5 ) / 2 );
}

namespace cube {
    
Eigen::Vector3d to_cartesian( const Eigen::Vector2d p, cube::faces::values face ) // todo! quick and dirty; debug
{
    Eigen::Vector3d v( p.x(), p.y(), 0 );
    v -= Eigen::Vector3d::Ones() * 0.5;
    switch( face )
    {
        case cube::faces::top: return Eigen::Vector3d( v.y(), v.x(), -v.z() );
        case cube::faces::back: return Eigen::Vector3d( -v.z(), -v.x(), v.y() );
        case cube::faces::left: return Eigen::Vector3d( v.x(), -v.z(), v.y() );
        case cube::faces::front: return Eigen::Vector3d( v.z(), v.x(), v.y() );
        case cube::faces::right: return Eigen::Vector3d( -v.x(), v.z(), v.y() );
        case cube::faces::bottom: return Eigen::Vector3d( -v.y(), v.x(), v.z() );
    }
    COMMA_THROW( comma::exception, "expected face enumeration value between 0 and 5; got: " << face );
}
    
} // namespace cube {

} } // namespace snark { namespace equirectangular {
