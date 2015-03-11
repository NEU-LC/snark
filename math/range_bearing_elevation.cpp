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

#include <cmath>
#include <Eigen/Geometry>
#include <comma/math/compare.h>
#include <snark/math/range_bearing_elevation.h>

namespace snark {

Eigen::Vector2d bearing::to_cartesian( const double radians, const double radius )
{
    return Eigen::Vector2d( radius * std::sin( radians ), radius * std::cos( radians ) );
}

double bearing::from_cartesian( const double x, const double y )
{
    double radians = ( M_PI / 2 ) - std::atan2( y, x );
    return radians < 0 ? radians + 2 * M_PI : radians;
}

bearing_elevation::bearing_elevation() : bearing_( 0 ), elevation_( 0 ) {}

bearing_elevation::bearing_elevation( double b, double e ) : bearing_( bearing( b ) ), elevation_( elevation( e ) ) {}

double bearing_elevation::bearing() const { return bearing_; }

double bearing_elevation::elevation() const { return elevation_; }

double bearing_elevation::b() const { return bearing(); }

double bearing_elevation::e() const { return elevation(); }

double bearing_elevation::b( double b ) { return bearing( b ); }

double bearing_elevation::e( double e ) { return elevation( e ); }

static double mod_( double t, double m ) // quick and dirty, because std::fmod is ridiculously slow
{
    int r = std::abs( t / m );
    return comma::math::less( t, 0 ) ? ( t + m * r ) : ( t - m * r );
}

double bearing_elevation::bearing( double t )
{
    double b( mod_( t, ( double )( M_PI * 2 ) ) ); //double b( std::fmod( t, ( double )( M_PI * 2 ) ) );
    if( !comma::math::less( b, M_PI ) ) { b -= ( M_PI * 2 ); }
    else if( comma::math::less( b, -M_PI ) ) { b += ( M_PI * 2 ); }
    bearing_ = b;
    return bearing_;
}

double bearing_elevation::elevation( double t )
{
    double e( mod_( t, ( double )( M_PI * 2 ) ) ); //double e( std::fmod( t, ( double )( M_PI * 2 ) ) );
    if( comma::math::less( e, 0 ) ) { e += M_PI * 2; }
    if( !comma::math::less( e, M_PI / 2 ) )
    {
        if( !comma::math::less( e, M_PI ) )
        {
            if( !comma::math::less( e, M_PI * 3 / 2 ) )
            {
                e = e - M_PI * 2;
            }
            else
            {
                e = M_PI - e;
                bearing( bearing() + M_PI );
            }
        }
        else
        {
            e = M_PI - e;
            bearing( bearing() + M_PI );
        }
    }
    elevation_ = e;
    return elevation_;
}

range_bearing_elevation::range_bearing_elevation( double r, double b, double e )
{
    bearing_elevation_.bearing( b );
    bearing_elevation_.elevation( e );
    range( r );
}

double range_bearing_elevation::range( double t )
{
    if( !comma::math::less( t, 0 ) )
    {
        range_ = t;
    }
    else
    {
        range_ = -t;
        bearing_elevation_.bearing( -bearing_elevation_.bearing() );
        bearing_elevation_.elevation( -bearing_elevation_.elevation() );
    }
    return range_;
}

Eigen::Vector3d range_bearing_elevation::to_cartesian() const
{
    double xy_projection( range() * std::cos( elevation() ) );
    return ::Eigen::Matrix< double, 3, 1 >( xy_projection * std::cos( bearing() )
                                    , xy_projection * std::sin( bearing() )
                                    , range() * std::sin( elevation() ) );
}

const range_bearing_elevation& range_bearing_elevation::from_cartesian( double x, double y, double z )
{
    return from_cartesian( Eigen::Vector3d( x, y, z ) );
}

const range_bearing_elevation& range_bearing_elevation::from_cartesian( const Eigen::Vector3d& v )
{
    range_ = v.norm();
    if( comma::math::equal( range_, 0 ) ) { bearing_elevation_ = bearing_elevation( 0, 0 ); return *this; }
    const Eigen::AngleAxis< double >& a =  Eigen::AngleAxis< double >( Eigen::Quaternion< double >::FromTwoVectors( v, Eigen::Vector3d( 0, 0, 1 ) ) );
    double e = M_PI / 2 - a.angle();
    Eigen::AngleAxis< double > c( -e, a.axis() );
    const Eigen::Matrix3d& r = c.toRotationMatrix();
    Eigen::AngleAxis< double > d( Eigen::Quaternion< double >::FromTwoVectors( Eigen::Vector3d( 1, 0, 0 ), r * v ) );
    bearing_elevation_.bearing( d.angle() * ( d.axis().z() < 0 ? -1 : 1 ) );
    bearing_elevation_.elevation( e );
    return *this;
}

// const range_bearing_elevation& range_bearing_elevation::from_cartesian( const Eigen::Vector3d& xyz )
// {
//     return from_cartesian( xyz[0], xyz[1], xyz[2] );
// }
//
// const range_bearing_elevation& range_bearing_elevation::from_cartesian( double x, double y, double z )
// { // todo: use rotation matrices instead!
//     long double projection_square ( x * x +  y * y );
//     if ( comma::math::equal( projection_square, 0 ) )
//     {
//         if ( comma::math::equal( z, 0 ) )
//         {
//             range_ = 0;
//             bearing_elevation_.bearing( 0 );
//             bearing_elevation_.elevation( 0 );
//             return *this;
//         }
//         range_ = std::abs( z );
//         bearing_elevation_.bearing( 0 );
//         elevation( comma::math::less( z, 0 ) ? -M_PI / 2 : M_PI / 2 );
//         return *this;
//     }
//     long double range_square( projection_square + z * z );
//     long double lr( std::sqrt( range_square ) );
//     long double elevation_ = 0;
//     if ( !comma::math::equal( z, 0 ) )
//     {
//         long double r = z / lr;
//         if ( comma::math::less( ( long double )( 1.0 ), r ) ) { r = 1; }
//         else if ( comma::math::less( r, ( long double ) ( -1.0 ) ) ) { r = -1; }
//         elevation_ = std::asin( r );
//     }
//     long double r = x / std::sqrt( projection_square );
//     if ( comma::math::less( ( long double )( 1.0 ), r ) ) { r = 1; }
//     else if ( comma::math::less( r, ( long double ) ( -1.0 ) ) ) { r = -1; }
//     long double bearing_ = std::acos( r );
//     if ( comma::math::less( y, 0 ) ) { bearing_ = M_PI * 2 - bearing_; }
//     range_ = lr;
//     bearing( bearing_ );
//     elevation( elevation_ );
//     return *this;
// }

Eigen::AngleAxis< double > great_circle_angle_axis( const bearing_elevation& lhs, const bearing_elevation& rhs )
{
    Eigen::Vector3d a = rbe( 1, lhs.bearing(), lhs.elevation() ).to_cartesian();
    Eigen::Vector3d b = rbe( 1, rhs.bearing(), rhs.elevation() ).to_cartesian();
    return Eigen::AngleAxis< double >( Eigen::Quaternion< double >::FromTwoVectors( a, b ) );
}

} // namespace snark {
