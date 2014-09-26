#include <iostream>
#include <gtest/gtest.h>
#include "../polytope.h"

using namespace snark::geometry;

TEST(geometry, convex_polytope_2d)
{
    Eigen::MatrixXd A(3,2);
    Eigen::VectorXd b(3);

    A<<1,0,
       0,1,
       -1,-1;
    b<<0,0,-1;
    convex_polytope poly(A,b);

    Eigen::Vector2d x;
    x<<0.1,0.1;

    EXPECT_EQ(true,poly.has(x));

    x<<1,1,1;

    EXPECT_EQ(false,poly.has(x));
}

TEST(geometry, convex_polytope_3d)
{
    Eigen::MatrixXd A(4,3);
    Eigen::VectorXd b(4);

    A<<1,0,0,
       0,1,0,
       0,0,1,
       -1,-1,-1;
    b<<0,0,0,-1;
    convex_polytope poly(A,b);

    Eigen::Vector3d x;
    x<<0.1,0.1,0.1;

    EXPECT_EQ(true,poly.has(x));

    x<<1,1,1;

    EXPECT_EQ(false,poly.has(x));
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
