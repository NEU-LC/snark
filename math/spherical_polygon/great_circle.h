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

#ifndef SNARK_MATH_SPHERICAL_POLYGON_GREAT_CIRCLE_H_
#define SNARK_MATH_SPHERICAL_POLYGON_GREAT_CIRCLE_H_

#include "./point.h"

namespace snark { namespace math {

/**
 * @class great_circle
 *
 *     @brief  A greate circle is a circle on a sphere whose origin is at the centre of the sphere
 *
 *     A greate circle is a circle on a sphere whose origin is at the centre of the sphere.
 *     The shortest distance between any two points on a sphere is the great circle distance.
 *     Any two points on a sphere uniquely define a single great circle so long as they are not antipodal.
 */
class great_circle
{
public:
    // Constructor
    great_circle( const point& start, const point& end );

    // Find intersection points of this and another great circle
    // - make sure the two planes have non-zero parameters.
    //   if there are zero parameters, transform it into another coordinate frame.
    //   the angle of transformation is chosen randomly, repeat until reach non-zero parameters.
    // - solve intersection points in 3D Cartesian coordinate system.
    // - inverse transform points back to original coordinate frame.
    bool intersects( const great_circle& other, double radius = point::EARTH_RADIUS_KM );

protected:
    void init_cartesian_plane_params();

    // Cartesian plane parameters in equation ax + by + cz = 0
    double a, b, c;

    // Points that define the great circle (arc endpoints)
    point arc_point[2];

    // Points for any intersectinos found between this and another great circle
    point intersect_point[2];
};

} } // namespace snark{ namespace math{

#endif // SNARK_MATH_SPHERICAL_POLYGON_GREAT_CIRCLE_H_

