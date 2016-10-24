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
#include <boost/format.hpp>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "../angle.h"
#include "coordinates.h"
#include <iomanip>

namespace {
    bool is_far( const snark::spherical::coordinates & l, const snark::spherical::coordinates & r, double _epsilon )
    {
        // for very short arcs skip shortcuts and enforce thorough comparison because of numerical stability issues in Eigen
        // the actual value is about 6 m / earth radius; enough to make AngleAxis::angle() non-zero
        double threshold = std::max( snark::spherical::coordinates::epsilon, _epsilon );
        // make quick decisions first; rely on the order of evaluation to return early if can avoid the last, expensive call
        double delta_latitude = std::abs( l.latitude - r.latitude );
        double delta_longitude = std::abs( l.longitude - r.longitude );
        // nearly 360 delta means very close; take the shorter of the two possible arcs
        delta_longitude = delta_longitude > M_PI ? 2 * M_PI - delta_longitude : delta_longitude;
        // first branch: self-explanatory
        // second branch: constants are 80 degrees in radians, cos(80 degrees)
        //                ignore near-Pole regions
        //                take a lower limit on cos(latitude):
        //                    if the second latitude is lower, the arc is longer
        //                    if the second latitude is the same, delta times times cos(latitude) is larger then same delta times the lower limit of cos
        //                thus, the real arc length can be only larger than this estimate
        // third branch: shortcuts failed, do lengthy computations
        return delta_latitude > threshold ||
               ( std::max( std::abs( l.latitude ), std::abs( r.latitude ) ) < 1.39626340159546366152 && delta_longitude * .17364817766693034887 > threshold ) ||
               Eigen::AngleAxis< double >( Eigen::Quaternion< double >::FromTwoVectors( l.to_cartesian(), r.to_cartesian() ) ).angle() > threshold;
    }
}

namespace snark { namespace spherical {

const double coordinates::epsilon = 1e-6;

bool coordinates::operator==( const coordinates& rhs ) const
{
    return !is_far( *this, rhs, epsilon );
    // return is_near( rhs, epsilon );
}

bool coordinates::is_near( const coordinates& c, double epsilon ) const
{
    return !is_far( *this, c, epsilon );
    // double dist = std::abs( longitude - c.longitude );
    // return comma::math::equal( latitude, c.latitude, epsilon ) && comma::math::equal( 0, std::min( dist, 2 * M_PI - dist ), epsilon );
}

bool coordinates::is_near( const coordinates & l, const coordinates & r, double _epsilon )
{
    return !is_far( l, r, _epsilon );
}

/// limits longitude to [-PI, PI)
static double limit( const double longitude )
{
    // todo: this is just a quick patch, not a real fix; what if longitude equals e.g. -3*M_PI?
    // todo: seriously quick and dirty, to fix anti merridian crossing
    return longitude < -M_PI ? longitude + 2 * M_PI : longitude >= M_PI ? longitude - 2 * M_PI : longitude;
}

coordinates::coordinates( const double latitude, const double longitude ) : latitude( latitude ), longitude( limit( longitude ) )
{
}

coordinates::coordinates( const snark::bearing_elevation& be ) : latitude( be.elevation() ), longitude( limit( be.bearing() ) )
{
}

coordinates::coordinates( const Eigen::Vector3d& xyz )
{
    snark::range_bearing_elevation rbe( xyz );
    latitude = rbe.elevation();
    longitude = limit( rbe.bearing() );
}

Eigen::Vector3d to_navigation_frame( const coordinates& c, const Eigen::Vector3d& v )
{
    const Eigen::Matrix3d& r1 = Eigen::AngleAxis< double >( -c.longitude, Eigen::Vector3d( 0, 0, 1 ) ).toRotationMatrix();
    const Eigen::Matrix3d& r2 = Eigen::AngleAxis< double >( c.latitude + M_PI / 2, Eigen::Vector3d( 0, 1, 0 ) ).toRotationMatrix();
    return r2 * r1 * v;
}

coordinates& coordinates::operator+=( const coordinates& rhs )
{
    double d = latitude + rhs.latitude;
    static const double epsilon = 0.00005;
    if( comma::math::equal( d, M_PI / 2, epsilon ) ) { d = M_PI / 2; }
    else if( comma::math::equal( d, -M_PI / 2, epsilon ) ) { d = -M_PI / 2; }
    else if( d > M_PI / 2 || d < -M_PI / 2 ) { COMMA_THROW( comma::exception, "adding " << ( latitude * 180 / M_PI ) << " and " << ( rhs.latitude * 180 / M_PI ) << " gives invalid latitude of " << ( d * 180 / M_PI ) << " degress" ); }
    latitude = d;
    longitude += rhs.longitude;
    while( longitude < -M_PI ) { longitude += 2 * M_PI; } // brutal, but mod() is slower in most cases, i think
    while( this->longitude >= M_PI ) { longitude -= 2 * M_PI; } // brutal, but mod() is slower in most cases, i think
    return *this;
}

coordinates::operator std::string() const
{
    const std::pair< double, double >& c = to_degrees();
    return (boost::format("%.2f") % c.first ).str() + "," + (boost::format("%.2f") % c.second).str();
}

bool coordinates::is_near( const Eigen::Vector3d& c, double _epsilon ) const
{
    return ( to_cartesian() - c ).lpNorm<Eigen::Infinity>() < _epsilon;
}

coordinates coordinates::from_degrees(double latitude, double longitude)
{
    return coordinates( snark::math::radians( snark::math::degrees( latitude ) ).value,
                        snark::math::radians( snark::math::degrees( longitude ) ).value );
}

std::pair< double, double > coordinates::to_degrees() const
{
    return std::make_pair( snark::math::degrees( snark::math::radians( latitude ) ).value
                         , snark::math::degrees( snark::math::radians( longitude ) ).value );
}

} } // namespace snark { namespace spherical {
