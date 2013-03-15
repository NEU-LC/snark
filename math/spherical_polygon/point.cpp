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

#include "./point.h"
#include "../angle.h"
#include "../range_bearing_elevation.h"
#include <cmath>

namespace snark { namespace math {

const double point::earth_radius_km = 6367.435;
const double point::reasonable_distance_precision = 0.0001;

point::point(double _x, double _y, double _z)
: x(_x), y(_y), z(_z)
{

}

point::point( const Eigen::Vector3d& v )
: x(v[0]), y(v[1]), z(v[2])
{

}

bool point::operator == ( const point& p )
{
    return x == p.x && y == p.y && z == p.z;
}

bool point::approx_equal( const point& p, double precision )
{
    return normsq(p) < precision;
}

std::ostream& operator << ( std::ostream& os, const point& p )
{
    os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    return os;
}

double point::normsq(const point& p)
{
    double dx = x - p.x;
    double dy = y - p.y;
    double dz = z - p.z;

    return dx*dx + dy*dy + dz*dz;
}

// inverse transform 3D Cartesian point through
//    p'=L3(-psi)*L2(-theta)*L1(-phi)*p
point point::inv_transform(double phi, double theta, double psi)
{
    double tmpx, tmpy, tmpz;

    tmpy =  cos(-phi)*y + sin(-phi)*z;
    tmpz = -sin(-phi)*y + cos(-phi)*z; 
    y = tmpy;
    z = tmpz;

    tmpx = -sin(-theta)*z + cos(-theta)*x; 
    tmpz =  cos(-theta)*z + sin(-theta)*x;
    x = tmpx;
    z = tmpz;

    tmpx =  cos(-psi)*x + sin(-psi)*y;
    tmpy = -sin(-psi)*x + cos(-psi)*y;
    x = tmpx;
    y = tmpy;

    return *this;
}

point point::transform(double phi, double theta, double psi)
{
    double tmpx, tmpy, tmpz;
    tmpx =  cos(psi)*x + sin(psi)*y;
    tmpy = -sin(psi)*x + cos(psi)*y;
    x = tmpx;
    y = tmpy;

    tmpx =  cos(theta)*x - sin(theta)*z;
    tmpz =  sin(theta)*x + cos(theta)*z;
    x = tmpx;
    z = tmpz;

    tmpy =  cos(phi)*y + sin(phi)*z;
    tmpz = -sin(phi)*y + cos(phi)*z;
    y = tmpy;
    z = tmpz;

    return *this;
}

point point::from_rad_lat_lon(double rad_lat, double rad_lon, double radius)
{
    Eigen::Vector3d v = snark::range_bearing_elevation( radius, rad_lon, rad_lat ).to_cartesian();
    // double x = radius * cos(rad_lon) * cos(rad_lat);
    // double y = radius * sin(rad_lon) * cos(rad_lat);
    // double z = radius                * sin(rad_lat);
    return point( v );
}

point point::from_deg_lat_lon(double deg_lat, double deg_lon, double radius)
{
    double rad_lat = radians( degrees (deg_lat) ).value;
    double rad_lon = radians( degrees (deg_lon) ).value;
    return from_rad_lat_lon(rad_lat,rad_lon,radius);
}

double point::to_rad_lat_lon(double& rad_lat, double& rad_lon)
{
    snark::range_bearing_elevation rbe;
    rbe.from_cartesian( x, y, z );
    rad_lat = rbe.elevation();
    rad_lon = rbe.bearing();
    return rbe.range();
}

double point::to_deg_lat_lon(double& deg_lat, double& deg_lon)
{
    double rad_lat;
    double rad_lon;
    double radius = to_rad_lat_lon ( rad_lat, rad_lon );
    deg_lat = degrees ( radians (rad_lat) ).value;
    deg_lon = degrees ( radians (rad_lon) ).value;
    return radius;
}

} } // namespace snark { namespace math {
