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


#ifndef SNARK_MATH_SPHERICAL_POLYGON_POINT_H_
#define SNARK_MATH_SPHERICAL_POLYGON_POINT_H_

#include <iostream>
#include <comma/base/exception.h>
#include <Eigen/Core>

namespace snark { namespace math {

/**
 * @class point
 *
 *     @brief point on great circle centred on the centre of earth
 *
 *     A point on great circle in 3D Cartesian or Spherical coordinate
 */
class point
{
public:

    /**
     * average radius of the earth (km)
     */
    static const double earth_radius_km;

    /**
     * magic number to check if one point is approximately equal to another point
     */
    static const double reasonable_distance_precision;

    point( double _x=0, double _y=0, double _z=0 );
    point( const Eigen::Vector3d& v );

    /**
     * equality test and approximate equality test
     */
    bool operator == (const point& p);
    bool approx_equal (const point& p, double precision = reasonable_distance_precision );

    /**
     * @return the norm-square of this point relative to a given point p
     */
    double normsq(const point& p);

    /**
     * transform and inverse transform of point
     */
    point transform(double phi, double theta, double psi);
    point inv_transform(double phi, double theta, double psi);

    /**
     * conversion from Spherical coordinate representation of the point (degree/radian)
     */
    static point from_deg_lat_lon(double deg_lat, double deg_lon, double radius = earth_radius_km);
    static point from_rad_lat_lon(double rad_lat, double rad_lon, double radius = earth_radius_km);

    /**
     * conversion to Spherical coordinate representation (degree/radian)
     *
     * @return the modulus or the range of the vector in addition to (latitude, longitude)
     */
    double to_deg_lat_lon ( double& deg_lat, double& deg_lon );
    double to_rad_lat_lon( double& rad_lat, double& rad_lon );

    friend class great_circle;
    friend std::ostream& operator << ( std::ostream& stream, const point& p );

private:
    double x, y, z; // Cartesian coordinate of the point (km)
};

std::ostream& operator << ( std::ostream& stream, const point& p );

} } // namespace snark{ namespace math{

#endif // SNARK_MATH_SPHERICAL_POLYGON_POINT_H_

