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
        EXPECT_GT( filter.state().covariance.determinant(), previouscovariance.determinant() );
        previouscovariance = filter.state().covariance;
        filter.update(m);
        EXPECT_LT( filter.state().covariance.determinant(), previouscovariance.determinant() );
        previouscovariance = filter.state().covariance;
        Eigen::Vector4d diff = filter.state().state_vector - previousstate;
        EXPECT_TRUE( diff.array().abs().isApprox( diff.array() ) );
        previousstate = filter.state().state_vector;
    }
}

} } 

