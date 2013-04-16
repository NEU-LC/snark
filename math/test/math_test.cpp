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

TEST( math, range_bearing_elevation )
{
    // todo
}

} }

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

    