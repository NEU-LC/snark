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

#include <gtest/gtest.h>
#include <snark/math/interval.h>

namespace snark { namespace math {

TEST( math, interval )
{
    Eigen::Vector3d a( 0, 1, 2 );
    Eigen::Vector3d b( 3, 4, 5 );

    interval< double, 3 > i( a, b );

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

} }

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

    