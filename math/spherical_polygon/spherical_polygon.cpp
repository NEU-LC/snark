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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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
