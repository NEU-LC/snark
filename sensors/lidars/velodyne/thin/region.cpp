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

/// @author vsevolod vlaskine

#include <cassert>
#include <cmath>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "../../../../math/range_bearing_elevation.h"
#include "focus.h"

#include <iostream>

namespace snark {  namespace velodyne { namespace thin {

sector::sector() : bearing( comma::math::interval< double >( -180.0, 180.0 ), 0.0 ), ken( 360 ), range( 0 ) {}

sector::sector( double bearing, double ken, double range )
    : bearing( comma::math::interval< double >( -180.0, 180.0 ), bearing )
    , ken( ken )
    , range( range )
{
}

bool sector::has( double r, double b, double ) const
{
    if( !comma::math::equal( range, 0 ) && comma::math::less( range, r ) ) { return false; }
    comma::math::cyclic< double > a( comma::math::interval< double >( -180.0, 180.0 ), b );
    double diff = std::abs( ( bearing - a )() );
    return comma::math::less( diff, ken / 2 );
}

double sector::coverage() const { return comma::math::equal( range, 0 ) ? ken / 360 : ( range / 30 ) * ( ken / 360 ); } // quick and dirty

extents::extents( const Eigen::Vector3d& min, const Eigen::Vector3d& max ) : interval( min, max ) {}

extents::extents( const math::closed_interval< double, 3 >& interval ) : interval( interval ) {}

bool extents::has( double range, double bearing, double elevation ) const // quick and dirty, watch performance
{
    //std::cerr << "xyz: " << range_bearing_elevation( range, bearing, elevation ).to_cartesian().transpose() << std::endl;
    return interval.contains( range_bearing_elevation( range, bearing, elevation ).to_cartesian() );
}

double extents::coverage() const // todo: quick and dirty; by right need to take cross-section of the extents with a conic section
{
    double roughly_radius = ( interval.max() - interval.min() ).norm() / 2;
    static const double max_radius = 60; // quick and dirty: 60 metres
    return std::pow( roughly_radius / max_radius, 3 );
}

} } } // namespace snark {  namespace velodyne { namespace thin {
