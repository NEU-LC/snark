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
