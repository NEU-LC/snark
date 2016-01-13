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


#include <iostream>
#include <gtest/gtest.h>
#include <boost/foreach.hpp>
#include <snark/math/filter/kalman_filter.h>
#include <snark/math/filter/constant_speed.h>
#include <Eigen/Dense>

namespace snark{

namespace test
{

// basic test for constant speed model
// give position measurement that are approximately increasing,
// expect the state increasing
// expect the covariance increasing after predict, decreasing after update
TEST( constant_speed, simple )
{
    constant_speed<2>::model model( 0.2 );

    kalman_filter< constant_speed<2>::state, constant_speed<2>::model > filter( constant_speed<2>::state(), model );

    double deltaT = 0.1; // interval between measurements

    std::vector< Eigen::Vector2d > measurements( 10 );
    measurements[0] << 1,1;
    measurements[1] << 0.8,0.8;
    measurements[2] << 1.1,1.1;
    measurements[3] << 1,1;
    measurements[4] << 1.5,1.5;
    measurements[5] << 1.2,1.2;
    measurements[6] << 1.9,1.9;
    measurements[7] << 1.3,1.3;
    measurements[8] << 2.2,2.2;
    measurements[9] << 1.9,1.9;
    
    Eigen::Vector4d previousstate = Eigen::Vector4d::Zero();
    Eigen::Matrix4d previouscovariance = Eigen::Matrix4d::Zero();
    BOOST_FOREACH( const Eigen::Vector2d& meas, measurements )
    {
        constant_speed<2>::position m( meas, 0.3 );
        filter.predict( deltaT );
        EXPECT_GT( filter.state.covariance.determinant(), previouscovariance.determinant() );
        previouscovariance = filter.state.covariance;
        filter.update(m);
        EXPECT_LT( filter.state.covariance.determinant(), previouscovariance.determinant() );
        previouscovariance = filter.state.covariance;
        Eigen::Vector4d diff = filter.state.state_vector - previousstate;
        EXPECT_TRUE( diff.array().abs().isApprox( diff.array() ) );
        previousstate = filter.state.state_vector;
    }
}

} } 

