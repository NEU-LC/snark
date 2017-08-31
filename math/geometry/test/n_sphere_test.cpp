#include <iostream>
#include <gtest/gtest.h>
#include "../n_sphere.h"

TEST( geometry, n_sphere )
{
    using snark::geometry::n_sphere;
    Eigen::Vector3d center( 0.5, 0.5, 0.5 );
    double radius = 1;

    n_sphere s( center, radius );

    // x,y,z
    EXPECT_TRUE(  s.contains( Eigen::Vector3d( 0.5, 0.5, 0.5 ) ) );
    EXPECT_TRUE(  s.contains( Eigen::Vector3d( 1.077, 1.077, 1.077 ) ) );
    EXPECT_FALSE( s.contains( Eigen::Vector3d( 1.078, 1.078, 1.078 ) ) );
    EXPECT_TRUE(  s.contains( Eigen::Vector3d( -0.077, -0.077, -0.077 ) ) );
    EXPECT_FALSE( s.contains( Eigen::Vector3d( -0.078, -0.078, -0.078 ) ) );

    // x,y
    EXPECT_TRUE(  s.contains( Eigen::Vector3d( 1.207, 1.207, 0.5 ) ) );
    EXPECT_FALSE( s.contains( Eigen::Vector3d( 1.208, 1.208, 0.5 ) ) );
    EXPECT_TRUE(  s.contains( Eigen::Vector3d( -0.207, -0.207, 0.5 ) ) );
    EXPECT_FALSE( s.contains( Eigen::Vector3d( -0.208, -0.208, 0.5 ) ) );

    // x
    EXPECT_TRUE(  s.contains( Eigen::Vector3d( 1.50, 0.5, 0.5 ) ) );
    EXPECT_FALSE( s.contains( Eigen::Vector3d( 1.51, 0.5, 0.5 ) ) );
    EXPECT_TRUE(  s.contains( Eigen::Vector3d( -0.50, 0.5, 0.5 ) ) );
    EXPECT_FALSE( s.contains( Eigen::Vector3d( -0.51, 0.5, 0.5 ) ) );

    // intersects
    EXPECT_TRUE(  s.intersects( n_sphere( Eigen::Vector3d( 2, 0.5, 0.5 ), 0.5 ) ) );
    EXPECT_TRUE(  s.intersects( n_sphere( Eigen::Vector3d( 2, 0.5, 0.5 ), 0.501 ) ) );
    EXPECT_FALSE( s.intersects( n_sphere( Eigen::Vector3d( 2, 0.5, 0.5 ), 0.499 ) ) );
}
