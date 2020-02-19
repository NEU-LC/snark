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
#include "polygon.h"

namespace snark {

template < typename C > static Eigen::Vector3d normal_impl( const C& corners )
{
    const Eigen::Vector3d& cross = ( corners[1] - corners[0] ).cross( corners[0] - corners[2] );
    return cross / cross.norm();
}

template < typename C > static Eigen::Vector3d projection_impl( const C& corners, const Eigen::Vector3d& rhs )
{
    const Eigen::Vector3d& n = normal_impl( corners );
    return rhs - n * ( rhs - corners[0] ).dot( n );
}

// See https://blogs.msdn.microsoft.com/rezanour/2011/08/07/barycentric-coordinates-and-point-in-triangle-tests
static inline bool is_inside( const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Eigen::Vector3d& p )
{
    const Eigen::Vector3d& u = b - a;
    const Eigen::Vector3d& v = c - a;
    const Eigen::Vector3d& w = p - a;

    const Eigen::Vector3d& v_cross_w = v.cross( w );
    const Eigen::Vector3d& v_cross_u = v.cross( u );

    if( v_cross_w.dot( v_cross_u ) < 0 ) { return false; }

    const Eigen::Vector3d& u_cross_w = u.cross( w );
    const Eigen::Vector3d& u_cross_v = -v_cross_u;

    if( u_cross_w.dot( u_cross_v ) < 0 ) { return false; }

    double denom = u_cross_v.norm();
    double r = v_cross_w.norm();
    double t = u_cross_w.norm();

    return( r + t <= denom );
}

template < typename C > static bool includes_impl( const C& corners, const Eigen::Vector3d& rhs )
{
    for( std::size_t i = 2; i < corners.size(); ++i )
    {
        if( is_inside( corners[0], corners[i-1], corners[i], rhs ) ) { return true; }
    }
    return false;
}

static inline double distance_to_line( const Eigen::Vector3d from, const Eigen::Vector3d& to, const Eigen::Vector3d& point )
{
    typedef Eigen::ParametrizedLine< double, 3 > line_t;
    line_t line( from, ( to - from ).normalized() );
    Eigen::Vector3d projection = line.projection( point );
    bool is_between = ( ( projection - from ).squaredNorm() + ( projection - to ).squaredNorm() ) <= ( to - from ).squaredNorm();
    return is_between ? ( projection - point ).norm() : std::min( ( from - point ).norm(), ( to - point ).norm() );
}

double convex_polygon::distance_from_border_to( const Eigen::Vector3d& rhs ) const
{
    double distance = distance_to_line( corners.front(), corners.back(), rhs );
    for( unsigned int i = 1; i < corners.size(); ++i )
    {
        double d = distance_to_line( corners[ i - 1 ], corners[i], rhs );
        if( d < distance ) { distance = d; }
    }
    return distance;
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

bool triangle::is_valid() const
{
    if( corners.size() != 3 ) { return false; }
    const Eigen::Vector3d& cross = ( corners[1] - corners[0] ).cross( corners[0] - corners[2] );
    return !comma::math::equal( cross.norm(), 0 );
}

Eigen::Vector3d triangle::projection_of( const Eigen::Vector3d& rhs ) const { return projection_impl( corners, rhs ); }

bool triangle::includes( const Eigen::Vector3d& rhs ) const { return includes_impl( corners, rhs ); }

double triangle::circumscribing_radius() const
{
    COMMA_THROW( comma::exception, "todo" );
    const Eigen::Vector3d& a = corners[1] - corners[0];
    const Eigen::Vector3d& b = corners[2] - corners[1];
    const Eigen::Vector3d& c = corners[0] - corners[2];
    return a.norm() / ( std::sqrt( 1 - b.dot( c ) / ( b.squaredNorm() * c.squaredNorm() ) ) * 2 );
}

double triangle::area() const { return ( ( corners[1] - corners[0] ).cross( corners[2] - corners[0] ) ).norm() / 2; }

} // namespace snark {
