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


#include "./great_circle.h"

namespace snark { namespace math {

great_circle::great_circle(const point& start, const point& end)
{
    arc_point[0] = start;
    arc_point[1] = end;
    init_cartesian_plane_params();
}

void great_circle::init_cartesian_plane_params()
{
    // initialise the Cartesian plane parameters
    a = (arc_point[0].y * arc_point[1].z) - (arc_point[1].y * arc_point[0].z);
    b = (arc_point[1].x * arc_point[0].z) - (arc_point[0].x * arc_point[1].z);
    c = (arc_point[0].x * arc_point[1].y) - (arc_point[1].x * arc_point[0].y);
}

bool great_circle::intersects(const great_circle& other, double radius)
{
    const int MAX_COORD_TRANSFORM = 10000; // macimum number of loops for coordinate transform

    if (&other == NULL) return false; // other is null
    if (this == &other) return false; // other is self

    double a0 = a;
    double b0 = b;
    double c0 = c;
    double a1 = other.a;
    double b1 = other.b;
    double c1 = other.c;

    if ((a0==0 && b0==0 && c0==0) || (a1==0 && b1==0 && c1==0))
    {
        return false;   // every polygon has one additional edge at the end. this has to be removed.
    }

    double phi = 0.0;
    double theta = 0.0;
    double psi = 0.0;

    int count = 0;
    double k1 = a0*b1 - b0*a1;
    bool is_transform = false;
    while (a0==0 || b0==0 || c0==0 || a1==0 || b1==0 || c1==0 || k1==0)
    {
         is_transform = true;

         phi   = ((double) rand() / RAND_MAX) * M_PI;
         theta = ((double) rand() / RAND_MAX) * M_PI;
         psi   = ((double) rand() / RAND_MAX) * M_PI;

         point tmp(a,b,c);
         tmp.transform(phi,theta,psi);
         a1 = tmp.x;
         b1 = tmp.y;
         c1 = tmp.z;

         k1 = a0*b1 - b0*a1;

         if (count++ == MAX_COORD_TRANSFORM)
         {
             return false;   // cannot find intersection
         }
    }

    double k = (c0*a1 - a0*c1)/k1;
    double m = (b0*c1 - c0*b1)/k1;

    double zz = radius/sqrt(m*m+k*k+1);
    double yy = k*zz;
    double xx = m*zz;

    intersect_point[0] = point(xx,yy,zz);
    intersect_point[1] = point(-xx,-yy,-zz);
    if (is_transform)
    {
        intersect_point[0].inv_transform(phi,theta,psi);
        intersect_point[1].inv_transform(phi,theta,psi);
    }

    return true;
}

} } // namespace snark { namespace math {
