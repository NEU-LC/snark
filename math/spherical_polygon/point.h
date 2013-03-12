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

