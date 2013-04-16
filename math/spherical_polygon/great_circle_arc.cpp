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
