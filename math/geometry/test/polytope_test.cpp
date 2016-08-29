// #include <iostream>
// #include <gtest/gtest.h>
// #include "../polytope.h"
// 
// using namespace snark::geometry;
// 
// TEST(geometry, convex_polytope_2d)
// {
//     Eigen::MatrixXd A(3,2);
//     Eigen::VectorXd b(3);
// 
//     A<<1,0,
//        0,1,
//        -1,-1;
//     b<<0,0,-1;
//         
//     convex_polytope poly(A,b);
//     
//     Eigen::Vector2d x;
//     x<<0.1,0.1;
// 
//     EXPECT_TRUE(poly.has(x));
//     
//     x<<1,1;
//     
//     EXPECT_FALSE( poly.has(x));
// }
// 
// TEST(geometry, convex_polytope_3d)
// {
//     Eigen::MatrixXd A;
//     Eigen::VectorXd b;
//     Eigen::Vector3d x;
// 
//     //first polytope, two points
//     A.resize(4,3);
//     b.resize(4);
//     A<<1,0,0,
//        0,1,0,
//        0,0,1,
//        -1,-1,-1;
//     b<<0,0,0,-1;
// 
//     x<<0.1,0.1,0.1;
// 
//     EXPECT_TRUE(convex_polytope(A,b).has(x));
// 
//     x<<1,1,1;
// 
//     EXPECT_FALSE(convex_polytope(A,b).has(x));
// 
//     //second polytope, two points
//     A.resize(6,3);
//     b.resize(6);
//     A<<1,0,0,
//        0,1,0,
//        0,0,1,
//        -1,0,0,
//        0,-1,0,
//        0,0,-1;
//     b<<0,0,0,-1,-1,-1;
// 
//     x<<0.5,0.5,0.5;
// 
//     EXPECT_TRUE(convex_polytope(A,b).has(x));
// 
//     x<<2,2,2;
// 
//     EXPECT_FALSE(convex_polytope(A,b).has(x));
// }
// 
// int main(int argc, char *argv[])
// {
//     ::testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }
