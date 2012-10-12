// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#include <cassert>
#include <cmath>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "./focus.h"

namespace snark {  namespace velodyne { namespace thin {

sector::sector() : bearing( comma::math::interval< double >( -180.0, 180.0 ), 0.0 ), ken( 0 ), range( 0 ) {}

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

} } } // namespace snark {  namespace velodyne { namespace thin {
