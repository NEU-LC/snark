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

