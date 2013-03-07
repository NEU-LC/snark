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

#include "./great_circle_arc.h"

namespace snark { namespace math {

great_circle_arc::great_circle_arc( const point& start, const point& end )
: great_circle( start, end )
{

}

bool great_circle_arc::in_arc( const point& p )
{
    // Get straight line distances
    double dP1P2sq = arc_point[0].normsq( arc_point[1] );
    double dS1P1sq = arc_point[0].normsq( p );
    double dS1P2sq = arc_point[1].normsq( p );

    // Triangular inequality rules apply, for points along arcs
    return ( dS1P1sq <= dP1P2sq && dS1P2sq <= dP1P2sq );
}

bool great_circle_arc::intersects_arc( const great_circle_arc& other )
{
    if ( !this->intersects( other ) )
    {
        return false;   // if there is no intersections, one possibility is they are the same circle.
    }

    great_circle_arc that = other;
    for ( int i=0; i<2; i++ )
    {
        // check if any of the two intersection points is within both great circle arcs
        if ( this->in_arc( intersect_point[i] ) && that.in_arc( intersect_point[i] ) )
        {
            return true;
        }
    }

    return false;
}

} } // namespace snark { namespace math {
