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


#include <gtest/gtest.h>
#include <snark/math/interval.h>
#include <snark/math/range_bearing_elevation.h>

namespace snark { namespace math {

TEST( math, closed_interval )
{
    Eigen::Vector3d a( 0, 1, 2 );
    Eigen::Vector3d b( 3, 4, 5 );

    closed_interval< double, 3 > i( a, b );

    EXPECT_TRUE( i.contains( a ) );
    EXPECT_TRUE( i.contains( b ) );
    EXPECT_TRUE( i.contains( 0.5*(a+b) ) );
    EXPECT_FALSE( i.contains( -a ) );

    i = i.hull( Eigen::Vector3d ( -1, 2, 3 ) );
    EXPECT_EQ( i.min(), Eigen::Vector3d( -1, 1, 2 ) );
    EXPECT_EQ( i.max(), Eigen::Vector3d( 3, 4, 5 ) );

    i = i.hull( Eigen::Vector3d ( 0, 10, 3 ) );
    EXPECT_EQ( i.min(), Eigen::Vector3d( -1, 1, 2 ) );
    EXPECT_EQ( i.max(), Eigen::Vector3d( 3, 10, 5 ) );
}

TEST( math, closed_interval_one_dimension_zero )
{
    Eigen::Vector3d a( 0, 1, 2 );
    Eigen::Vector3d b( 0, 4, 5 );
    closed_interval< double, 3 > i( a, b );
    EXPECT_TRUE( i.contains( a ) );
    EXPECT_TRUE( i.contains( b ) );
}

TEST( math, closed_interval_set_hull )
{
    {
        closed_interval< double, 3 > i;
        i.set_hull( Eigen::Vector3d ( 1, 2, 3 ) );
        EXPECT_EQ( i.min(), Eigen::Vector3d( 1, 2, 3 ) );
        EXPECT_EQ( i.max(), Eigen::Vector3d( 1, 2, 3 ) );
    }
    {
        closed_interval< double, 3 > i;
        closed_interval< double, 3 > j;
        j.set_hull( Eigen::Vector3d( 10, -10, 10 ) );
        j.set_hull( Eigen::Vector3d ( -10, 10, 10 ) );
        i.set_hull( j );
        EXPECT_EQ( i.min(), Eigen::Vector3d( -10, -10, 10 ) );
        EXPECT_EQ( i.max(), Eigen::Vector3d( 10, 10, 10 ) );
    }
}

TEST( math, closed_interval_contains )
{
    snark::math::closed_interval< double, 3 > i( Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 10, 10, 5 ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 10, 1, 1 ) ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 1, 1, 5 ) ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 0, 1, 1 ) ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 1, 0, 1 ) ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 1, 1, 0 ) ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 1, 1, 1 ) ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 0, 0, 0 ) ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 0, 0, 5 ) ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 0, 10, 0 ) ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 0, 10, 5 ) ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 10, 0, 0 ) ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 10, 0, 5 ) ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 10, 10, 0 ) ) );
    EXPECT_TRUE( i.contains( Eigen::Vector3d( 10, 10, 5 ) ) );

    EXPECT_TRUE( i.contains( Eigen::Vector3d( 1, 0, 0 ) ) );
}

TEST( math, bearing )
{
    {
        double radians = 0 * M_PI / 180;
        Eigen::Vector2d c = snark::bearing::to_cartesian( radians, 1 );
        EXPECT_NEAR( 0, c.x(), 1e-6 );
        EXPECT_NEAR( 1, c.y(), 1e-6 );
    }

    {
        double radians = 30 * M_PI / 180;
        Eigen::Vector2d c = snark::bearing::to_cartesian( radians, 1 );
        EXPECT_NEAR( 0.5, c.x(), 1e-6 );
        EXPECT_NEAR( 0.866025404, c.y(), 1e-6 );
    }

    {
        double radians = 120 * M_PI / 180;
        Eigen::Vector2d c = snark::bearing::to_cartesian( radians, 1 );
        EXPECT_NEAR( 0.866025404, c.x(), 1e-6 );
        EXPECT_NEAR( -0.5, c.y(), 1e-6 );
    }

    {
        double radians = 210 * M_PI / 180;
        Eigen::Vector2d c = snark::bearing::to_cartesian( radians, 1 );
        EXPECT_NEAR( -0.5, c.x(), 1e-6 );
        EXPECT_NEAR( -0.866025404, c.y(), 1e-6 );
    }

    {
        double radians = 300 * M_PI / 180;
        Eigen::Vector2d c = snark::bearing::to_cartesian( radians, 1 );
        EXPECT_NEAR( -0.866025404, c.x(), 1e-6 );
        EXPECT_NEAR( 0.5, c.y(), 1e-6 );
    }

    {
        double radians = 360 * M_PI / 180;
        Eigen::Vector2d c = snark::bearing::to_cartesian( radians, 1 );
        EXPECT_NEAR( 0, c.x(), 1e-6 );
        EXPECT_NEAR( 1, c.y(), 1e-6 );
    }

    EXPECT_NEAR(   0 * M_PI / 180, snark::bearing::from_cartesian( 0, 1 ), 1e-6 );
    EXPECT_NEAR(  30 * M_PI / 180, snark::bearing::from_cartesian( 0.5, 0.866025404 ), 1e-6 );
    EXPECT_NEAR( 120 * M_PI / 180, snark::bearing::from_cartesian( 0.866025404, -0.5 ), 1e-6 );
    EXPECT_NEAR( 210 * M_PI / 180, snark::bearing::from_cartesian( -0.5, -0.866025404 ), 1e-6 );
    EXPECT_NEAR( 300 * M_PI / 180, snark::bearing::from_cartesian( -0.866025404, 0.5 ), 1e-6 );
}

TEST( math, range_bearing_elevation )
{
    EXPECT_NEAR( -M_PI, snark::bearing_elevation( M_PI, 0 ).b(), 1e-6 );
    EXPECT_NEAR( 0, snark::bearing_elevation( M_PI * 2, 0 ).b(), 1e-6 );
    EXPECT_NEAR( -M_PI / 2, snark::bearing_elevation( M_PI * 1.5, 0 ).b(), 1e-6 );
    EXPECT_NEAR( 0, snark::bearing_elevation( M_PI * 20, 0 ).b(), 1e-6 );

    //snark::range_bearing_elevation rbe;
    //rbe.from_cartesian( Eigen::Vector3d( -0.907717, -0.160055, -0.0673507 ) );
    //static const double radian = M_PI / 180;
    //std::cout << "==================" << std::endl;
    //std::cout << "==> " << rbe.r() << "," << ( rbe.b() / radian ) << "," << ( rbe.e() / radian ) << std::endl;
    // todo: certainly more testing
}

TEST( math, great_circle_angle_axis )
{
    EXPECT_NEAR( 0, great_circle_angle_axis( bearing_elevation( 0, 0 ), bearing_elevation( 0, 0 ) ).angle(), 0.000001 );
    EXPECT_NEAR( 1, great_circle_angle_axis( bearing_elevation( 0, 0 ), bearing_elevation( 0, 1 ) ).angle(), 0.000001 );
    EXPECT_NEAR( 1, great_circle_angle_axis( bearing_elevation( 0, 0 ), bearing_elevation( 1, 0 ) ).angle(), 0.000001 );
    EXPECT_NEAR( M_PI, great_circle_angle_axis( bearing_elevation( 0, 0 ), bearing_elevation( M_PI, 0 ) ).angle(), 0.000001 );
    EXPECT_NEAR( M_PI / 2, great_circle_angle_axis( bearing_elevation( 0, 0 ), bearing_elevation( 0, M_PI / 2 ) ).angle(), 0.000001 );
    // todo: certainly more testing...
}

} }

