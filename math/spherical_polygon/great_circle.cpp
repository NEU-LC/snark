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
