#include <iostream>
#include <gtest/gtest.h>
#include "../polygon.h"

TEST(geometry, triangle)
{
    // Simple tests
    Eigen::Vector3d a( 0, 0, 0 );
    Eigen::Vector3d b( 0, 1, 0 );
    Eigen::Vector3d c( 1, 0, 0 );

    snark::triangle t( a, b, c );

    EXPECT_TRUE(  t.includes( Eigen::Vector3d( 0.5, 0.1, 0 )));
    EXPECT_TRUE(  t.includes( Eigen::Vector3d( 0.1, 0.5, 0 )));
    EXPECT_FALSE( t.includes( Eigen::Vector3d( 0.5, 1.0, 0 )));
    EXPECT_FALSE( t.includes( Eigen::Vector3d( 1.0, 0.5, 0 )));
    EXPECT_FALSE( t.includes( Eigen::Vector3d( 0.5, -0.1, 0 )));
    EXPECT_FALSE( t.includes( Eigen::Vector3d( -0.1, 0.5, 0 )));

    EXPECT_TRUE(  t.includes( Eigen::Vector3d( 0.5, 0.1, 0.01 )));
    EXPECT_FALSE( t.includes( Eigen::Vector3d( 0.5, 0.1, 0.5 )));

    // Move everything away from the origin
    a = Eigen::Vector3d( 1, 1, 1 );
    b = Eigen::Vector3d( 1, 2, 1 );
    c = Eigen::Vector3d( 2, 1, 1 );

    t = snark::triangle( a, b, c );

    EXPECT_TRUE(  t.includes( Eigen::Vector3d( 1.5, 1.1, 1 )));
    EXPECT_TRUE(  t.includes( Eigen::Vector3d( 1.1, 1.5, 1 )));
    EXPECT_FALSE( t.includes( Eigen::Vector3d( 1.5, 2.0, 1 )));
    EXPECT_FALSE( t.includes( Eigen::Vector3d( 2.0, 1.5, 1 )));
    EXPECT_FALSE( t.includes( Eigen::Vector3d( 1.5, 0.9, 1 )));
    EXPECT_FALSE( t.includes( Eigen::Vector3d( 0.9, 1.5, 1 )));

    EXPECT_TRUE(  t.includes( Eigen::Vector3d( 1.5, 1.1, 1.01 )));
    EXPECT_FALSE( t.includes( Eigen::Vector3d( 1.5, 1.1, 1.5 )));
}
