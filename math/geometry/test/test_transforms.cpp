#include <iostream>
#include <gtest/gtest.h>
#include "../transforms.h"
#include <boost/math/constants/constants.hpp>

using namespace snark;

TEST(transforms, dh_to_matrix)
{
    dh_transform T_dh;
    T_dh.alpha=1;
    T_dh.r=1;
    T_dh.theta=boost::math::constants::pi<double>()/6;
    T_dh.alpha=-boost::math::constants::pi<double>()/2;
    EXPECT_LT((dh_to_matrix(T_dh)-(Eigen::Matrix4d()<<0.866025,0,-0.5,0.866025,0.5,0,0.866025,0.5,0,-1,0,0,0,0,0,1).finished()).norm(),1e-2);

}

TEST(transforms, dh_to_tr)
{
    dh_transform T_dh;
    T_dh.alpha=1;
    T_dh.r=1;
    T_dh.theta=boost::math::constants::pi<double>()/6;
    T_dh.alpha=-boost::math::constants::pi<double>()/2;
    tr_transform T_tr=dh_to_tr(T_dh);
    EXPECT_LT((homogeneous_transform(T_tr.rotation.toRotationMatrix(),T_tr.translation)-(Eigen::Matrix4d()<<0.866025,0,-0.5,0.866025,0.5,0,0.866025,0.5,0,-1,0,0,0,0,0,1).finished()).norm(),1e-2);

}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
