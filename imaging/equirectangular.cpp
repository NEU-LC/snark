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
#include "../math/range_bearing_elevation.h"
#include "equirectangular.h"

namespace snark { namespace equirectangular {

faces::values face_of( const Eigen::Vector3d& v ) // todo: watch performance, don't need to call it so many times
{
    double ax = std::abs( v.x() );
    double ay = std::abs( v.y() );
    double az = std::abs( v.z() );
    if( ax > ay && ax > az ) { return v.x() > 0 ? faces::front : faces::back; }
    if( ay > az ) { return v.y() > 0 ? faces::right : faces::left; }
    return v.z() > 0 ? faces::bottom : faces::top;
}

std::pair< Eigen::Vector2d, faces::values > to_cube( const Eigen::Vector2d& p, double spherical_width )
{
    return to_cube( Eigen::Vector2d( p.x(), p.y() * 2 ) / spherical_width );
}

std::pair< Eigen::Vector2d, faces::values > to_cube( const Eigen::Vector2d& p )
{
    auto c = snark::range_bearing_elevation( 0.5, M_PI * 2 * ( p.x() - 0.5 ), M_PI * ( p.y() - 0.5 ) ).to_cartesian();
    std::pair< Eigen::Vector2d, faces::values > pixel;
    pixel.second = face_of( c );
    switch( pixel.second )
    {
        case faces::top: pixel.first = Eigen::Vector2d( c.y(), c.x() ) / std::abs( c.z() ); break;
        case faces::left: pixel.first = Eigen::Vector2d( c.x(), c.z() ) / std::abs( c.y() ); break;
        case faces::front: pixel.first = Eigen::Vector2d( c.y(), c.z() ) / std::abs( c.x() ); break;
        case faces::right: pixel.first = Eigen::Vector2d( -c.x(), c.z() ) / std::abs( c.y() ); break;
        case faces::back: pixel.first = Eigen::Vector2d( -c.y(), c.z() ) / std::abs( c.x() ); break;
        case faces::bottom: pixel.first = Eigen::Vector2d( c.y(), -c.x() ) / std::abs( c.z() ); break;
    }
    pixel.first *= 0.5;
    pixel.first.x() += 0.5;
    pixel.first.y() += 0.5;
    return pixel;
}

// Eigen::Vector3d face_to_cartesian( const Eigen::Vector3d& norm, faces::values face ) // quick and dirty
// {
//     switch( face )
//     {
//         case faces::top: return Eigen::Vector3d( norm.y(), norm.x(), -norm.z() );
//         case faces::back: return Eigen::Vector3d( -norm.z(), -norm.x(), norm.y() );
//         case faces::left: return Eigen::Vector3d( norm.x(), -norm.z(), norm.y() );
//         case faces::front: return Eigen::Vector3d( norm.z(), norm.x(), norm.y() );
//         case faces::right: return Eigen::Vector3d( -norm.x(), norm.z(), norm.y() );
//         case faces::bottom: return Eigen::Vector3d( -norm.y(), norm.x(), norm.z() );
//     }
// }

} } // namespace snark { namespace equirectangular {
