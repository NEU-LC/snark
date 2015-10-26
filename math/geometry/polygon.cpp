// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2014 The University of Sydney
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

#include <Eigen/Geometry>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include <Eigen/Eigen>
#include "polygon.h"

namespace snark {

template < typename C > static Eigen::Vector3d normal_impl( const C& corners )
{
    const Eigen::Vector3d& cross = ( corners[1] - corners[0] ).cross( corners[2] - corners[1] );
    return cross / cross.norm();
}

template < typename C > static Eigen::Vector3d projection_impl( const C& corners, const Eigen::Vector3d& rhs )
{
    const Eigen::Vector3d& n = normal_impl( corners );
    double d = ( rhs - corners[0] ).dot( n );
    return rhs - n * d;
}

template < typename C > static bool includes_impl( const C& corners, const Eigen::Vector3d& rhs )
{
    for( std::size_t i = 1; i < corners.size(); ++i ) { if( comma::math::less( ( corners[i] - corners[ i - 1 ] ).dot( rhs - corners[ i - 1 ] ), 0 ) ) { return false; } }
    return !comma::math::less( ( corners[0] - corners.back() ).dot( rhs - corners.back() ), 0 );
}
    
Eigen::Vector3d convex_polygon::normal() const { return normal_impl( corners ); }

bool convex_polygon::is_valid() const
{
    if( corners.size() < 3 ) { return false; }
    COMMA_THROW( comma::exception, "todo" );
    for( std::size_t i = 1; i < corners.size(); ++i )
    {
        // todo
    }
    return true;
}
    
Eigen::Vector3d convex_polygon::projection_of( const Eigen::Vector3d& rhs ) const { return projection_impl( corners, rhs ); }

bool convex_polygon::includes( const Eigen::Vector3d& rhs ) const { return includes_impl( corners, rhs ); }

Eigen::Vector3d triangle::normal() const { return normal_impl( corners ); }

Eigen::Vector3d triangle::projection_of( const Eigen::Vector3d& rhs ) const { return projection_impl( corners, rhs ); }

bool triangle::includes( const Eigen::Vector3d& rhs ) const { return includes_impl( corners, rhs ); }

} // namespace snark {
