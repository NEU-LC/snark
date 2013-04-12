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
    bool intersects( const great_circle& other, double radius = point::earth_radius_km );

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

