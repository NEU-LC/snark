// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2013 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
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
#include <cmath>
#include "../frame_transforms.h"

using namespace snark::frame_transforms;

TEST(transforms, dh_to_matrix)
{
    dh_transform T_dh;
    T_dh.alpha=1;
    T_dh.r=1;
    T_dh.theta=M_PI/6;
    T_dh.alpha=-M_PI/2;
    EXPECT_LT((dh_to_matrix(T_dh)-(Eigen::Matrix4d()<<0.866025,0,-0.5,0.866025,0.5,0,0.866025,0.5,0,-1,0,0,0,0,0,1).finished()).norm(),1e-2);

}

TEST(transforms, dh_to_tr)
{
    dh_transform T_dh;
    T_dh.alpha=1;
    T_dh.r=1;
    T_dh.theta=M_PI/6;
    T_dh.alpha=-M_PI/2;
    tr_transform T_tr=dh_to_tr(T_dh);
    EXPECT_LT((homogeneous_transform(T_tr.rotation.toRotationMatrix(),T_tr.translation)-(Eigen::Matrix4d()<<0.866025,0,-0.5,0.866025,0.5,0,0.866025,0.5,0,-1,0,0,0,0,0,1).finished()).norm(),1e-2);

}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
