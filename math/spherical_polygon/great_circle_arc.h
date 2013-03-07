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

#ifndef SNARK_MATH_SPHERICAL_POLYGON_GREAT_CIRCLE_ARC_H_
#define SNARK_MATH_SPHERICAL_POLYGON_GREAT_CIRCLE_ARC_H_

#include "./great_circle.h"

namespace snark{ namespace math{

/**
 * @class great_circle_arc
 *
 *     @brief A great circle arc is a subsection of a great circle.
 *
 *     A great circle arc is a subsection of a great circle.
 */
class great_circle_arc : public great_circle
{
public:
    great_circle_arc (const point& start, const point& end);

    /**
     * Check if any of the intersection points are inside this.arc and other.arc
     * @param other great_circle_arc
     * @return true if other intersects with the arc
     */
    bool intersects_arc( const great_circle_arc& other );

private:

    /**
     * Check if a point p is along the arc
     * It uses distance equation, i.e., if the point is inside arc AB,
     * then dist(pA) and dist(pB) has to be less than dist(AB).
     * @param p input point to test
     * @return true if p is along the arc
     */
    bool in_arc(const point& p);
};

} } // namespace snark{ namespace math{

#endif // SNARK_MATH_SPHERICAL_POLYGON_GREAT_CIRCLE_ARC_H_

