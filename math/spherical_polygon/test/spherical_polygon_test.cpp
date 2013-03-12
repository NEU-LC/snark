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

#include <iostream>
#include <gtest/gtest.h>
#include <boost/foreach.hpp>
#include <snark/math/spherical_polygon/point.h>
#include <snark/math/spherical_polygon/great_circle.h>
#include <snark/math/spherical_polygon/great_circle_arc.h>
#include <snark/math/spherical_polygon/spherical_polygon.h>

namespace snark {

namespace test
{

// basic test for spherical_polygon class
TEST( spherical_polygon, simple )
{
    // test point class
    double lat = 0, lon = 0, ran = 0;
    math::point o(0,0,0);
    math::point p = math::point::from_deg_lat_lon(60,  20);
    math::point q = math::point::from_deg_lat_lon(60, 120);
    math::point r = math::point::from_deg_lat_lon(60, 220);
    math::point w = math::point::from_deg_lat_lon(30, 100);

    ran = p.to_deg_lat_lon(lat, lon);
    // std::cout << "lat " << lat << ", lon " << lon << ", ran " << ran << std::endl;
    EXPECT_TRUE ( p.approx_equal( math::point::from_deg_lat_lon(lat, lon, ran) ) );

    math::point t ( p ); // copy p into t
    math::point s = p.transform(0,0,0); // copy p transformed into s
    EXPECT_TRUE( s == p && s == t ); // p is null transformed, so all should be equal

    t = q; // copy q into t
    // std::cout << "q " << q << std::endl;
    s = q.transform(10, 10, 10); // q transformed
    // std::cout << "q transformed " << q << std::endl;
    EXPECT_TRUE( s == q );
    EXPECT_FALSE( s == t || s.approx_equal(t) );

    s = q.inv_transform(10, 10, 10); // q inverse transformed
    // std::cout << "q inv_transformed " << q << std::endl;
    // std::cout << "t (original q) " << t << std::endl;
    EXPECT_TRUE( s == q );
    EXPECT_TRUE( s == t || s.approx_equal(t) ); // q should be back to original value

    s = q.transform(10, 10, 10); // q transformed
    s = q.transform(-10, -10, -10); // q transformed backwards but not inverse transform
    EXPECT_FALSE( q == t ); // q should not be back to original value

    math::great_circle     l_gc  (p, q);
    math::great_circle_arc l_gc_a(p, q);

    std::vector<math::point> points;
    points.push_back(p);
    points.push_back(q);
    points.push_back(r);
    math::spherical_polygon poly(points);
    math::point e = math::point::from_deg_lat_lon(60, 60, 2*math::point::EARTH_RADIUS_KM);
    poly.set_external_point(e);

    EXPECT_FALSE( poly.contains (e) );
    EXPECT_FALSE( poly.contains (r) );
    EXPECT_TRUE ( poly.contains (w) );
    EXPECT_FALSE( poly.intersects (p, q) );
    EXPECT_FALSE( poly.intersects (s, t) );
    EXPECT_TRUE ( poly.intersects (w, t) );
}

} } 

