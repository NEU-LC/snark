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
    math::point e = math::point::from_deg_lat_lon(60, 60, 2*math::point::earth_radius_km);
    poly.set_external_point(e);

    EXPECT_FALSE( poly.contains (e) );
    EXPECT_FALSE( poly.contains (r) );
    EXPECT_TRUE ( poly.contains (w) );
    EXPECT_FALSE( poly.intersects (p, q) );
    EXPECT_FALSE( poly.intersects (s, t) );
    EXPECT_TRUE ( poly.intersects (w, t) );
}

} } 

