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

#include "./spherical_polygon.h"
#include "./great_circle_arc.h"

namespace snark { namespace math {

spherical_polygon::spherical_polygon( const std::vector<point>& points )
{
    corner_point = points;
}

void spherical_polygon::set_external_point( const point& p )
{
    external_point = p;
}

bool spherical_polygon::intersects( const point& from, const point& to )
{
    great_circle_arc arc(from, to);

    // determine if the great circle arc (from, to) crosses the polygon
    // by going through every arc edge of polygon to perform intersection test
    for ( size_t i=0; i<corner_point.size() - 1; i++ )
    {
        // if from or to node is one of the corner point, force no-intersect
        // this is valid since we have previously checked all points are outside of polygon
        if ( corner_point[i].approx_equal(to) || corner_point[i+1].approx_equal(to) )
            continue;
        if ( corner_point[i].approx_equal(from) || corner_point[i+1].approx_equal(from) )
            continue;
        if ( arc.intersects_arc( great_circle_arc( corner_point[i], corner_point[i+1] ) ) )
            return true;
    }

    return false; // no intersection found
}

bool spherical_polygon::contains(const point& p)
{
    // Determine if the polygon contains a given point by checking if an edge constructed
    // from the point to the external_point crosses the polygon
    great_circle_arc arc(p, external_point);
    int count = 0;
    for ( size_t i=0; i<corner_point.size() - 1; i++ )
    {
        if ( arc.intersects_arc( great_circle_arc( corner_point[i], corner_point[i+1] ) ) )
        {
            ++count;
        }
    }

    // if count is odd the point is inside polygon, outside otherwise.
    return (count % 2 == 1);
}

} } // namespace snark { namespace math {
