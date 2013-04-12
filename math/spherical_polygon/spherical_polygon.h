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


#ifndef SNARK_MATH_SPHERICAL_POLYGON_SPHERICAL_POLYGON_H_
#define SNARK_MATH_SPHERICAL_POLYGON_SPHERICAL_POLYGON_H_

#include <vector>
#include "./point.h"

namespace snark { namespace math {

/**
 * @class spherical_polygon
 *
 *     @brief A polygon on a sphere with corner points connected by great circle arcs.
 *
 *     repacked from nsidc by Seng keat Gan SKG (7/12/2012)
 *     repacked into C++ (7/3/2013)
 *
 *     SphericalPolygon connects corner points with the shortest great circle arc and
 *     make no assumptions about convexity. A poorly defined spherical polygon may have
 *     sides that cross.
 *     An external point needs to be supplied for point inside polygon test.
 *     No automatic method is implemented for generating external points to the polygon.
 */
class spherical_polygon
{
public:
    /**
     * constructs a spherical polygon from given vector of corner points.
     *
     * @param points the corner points connected by great circle arcs to form the spherical polygon
     */
    spherical_polygon( const std::vector<point> &points );

    /**
     * set the external point to be used for point inside polygon test
     *
     * @param p the supplied external point to the spherical polygon
     */
    void set_external_point( const point& p );

    /**
     * check if the spherical polygon intersects with a great circle arc (from, to)
     *
     * @param from start point  of the great circle arc to test for intersection
     * @param to     end point  of the great circle arc to test for intersection
     * @return true if the arc (from, to) intersects with the polygon
     */
    bool intersects( const point& from, const point& to );

    /**
     * check if the supplied point is inside the spherical polygon
     *
     * @param p the supplied point to be tested for inside/outside polygon
     * @return true if point is inside polygon
     */
    bool contains( const point& p );

private:
    /**
     * A vector of polygon vertices
     */
    std::vector<point> corner_point;

    /**
     * A point outside the polygon
     */
    point external_point;
};

} } // namespace snark { namespace math {

#endif // SNARK_MATH_SPHERICAL_POLYGON_SPHERICAL_POLYGON_H_
