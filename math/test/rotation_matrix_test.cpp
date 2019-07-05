// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2019 The University of Sydney
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

#include "../rotation_matrix.h"
#include <comma/math/compare.h>
#include <gtest/gtest.h>
#include <cmath>

namespace snark { namespace test {

double deg2rad( double degrees ) { return degrees * M_PI / 180; }

// Test conversion to and from roll, pitch, yaw format.
// To test we run through both conversions and compare results.

::testing::AssertionResult test_rpy( double roll, double pitch, double yaw )
{
    Eigen::Vector3d rpy( deg2rad( roll ), deg2rad( pitch ), deg2rad( yaw ));
    rotation_matrix m( rpy );
    Eigen::Quaterniond input_q = m.quaternion();

    Eigen::Vector3d rpy_after_conversions = m.roll_pitch_yaw();
    Eigen::Quaterniond result_q = rotation_matrix( rpy_after_conversions ).quaternion();

    // Quaternions might not be identical but if they represent equivalent
    // rotations then their dot product will be 1 or -1
    if( comma::math::equal( std::fabs( input_q.dot( result_q )), 1, 1e-12 ))
        return ::testing::AssertionSuccess();
    else
        return ::testing::AssertionFailure() << "\nin:\n" << rpy << "\nout:\n" << rpy_after_conversions
                                             << "\nquaternion dot product: " << input_q.dot( result_q );
}

TEST( rotation_matrix, roll_pitch_yaw )
{
    std::vector< double > angles{ -180, -120, -90, -30, 0, 30, 90, 120, 180 };

    for( auto roll : angles )
    {
        for( auto pitch : angles )
        {
            for( auto yaw : angles )
            {
                EXPECT_TRUE( test_rpy( roll, pitch, yaw ));
            }
        }
    }
}

} } // namespace snark { namespace test {
