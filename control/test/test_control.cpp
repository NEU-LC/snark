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

#include <gtest/gtest.h>
#include <comma/base/exception.h>
#include "../applications/pid.h"

namespace snark { namespace test {

TEST( pid, threshold_throw )
{
    ASSERT_THROW( snark::control::pid<>( 0, 0, 0, -1.0 ), comma::exception );
    ASSERT_THROW( snark::control::pid<>( 0, 0, 0, 0.0 ), comma::exception );
}

TEST( pid, zero_time_increment_throw )
{
    snark::control::pid<> pid( 0, 0, 0 );
    boost::posix_time::ptime t = boost::posix_time::from_iso_string( "20101010T121212.123456" );
    pid.update( 0, t );
    ASSERT_THROW( pid.update( 0, t ), comma::exception );
}

TEST( pid, proportional )
{
    const double p = 0.12;
    snark::control::pid<> pid( p, 0, 0 );
    boost::posix_time::ptime t = boost::posix_time::from_iso_string( "20101010T121212.123456" );
    double error = 0.1;
    EXPECT_EQ( p*error, pid.update( error, t ) );
    t += boost::posix_time::seconds( 1 );
    error = 0.11;
    EXPECT_EQ( p*error, pid.update( error, t ) );
    t += boost::posix_time::seconds( 2 );
    error = 0.12;
    EXPECT_EQ( p*error, pid.update( error, t ) );
}

TEST( pid, integral )
{
    const double i = 0.04;
    snark::control::pid<> pid( 0, i, 0 );
    boost::posix_time::ptime t = boost::posix_time::from_iso_string( "20101010T121212.123456" );
    double error = 0.1;
    double integral = 0;
    EXPECT_EQ( i*integral, pid.update( error, t ) );
    int dt = 1;
    t += boost::posix_time::seconds( dt );
    error = 0.11;
    integral += error * dt;
    EXPECT_EQ( i*integral, pid.update( error, t ) );
    dt = 2;
    t += boost::posix_time::seconds( dt );
    error = -0.12;
    integral += error * dt;
    EXPECT_EQ( i*integral, pid.update( error, t ) );
}

TEST( pid, threshold )
{
    const double i = 0.1;
    const double threshold = 0.15;
    snark::control::pid<> pid( 0, i, 0, threshold );
    boost::posix_time::ptime t = boost::posix_time::from_iso_string( "20101010T121212.123456" );
    double error = 0;
    EXPECT_EQ( 0, pid.update( error, t ) );
    int dt = 2;
    t += boost::posix_time::seconds( dt );
    error = -0.1;
    EXPECT_EQ( i*(-threshold), pid.update( error, t ) );
}

TEST( pid, derivative )
{
    const double d = 0.1;
    snark::control::pid<> pid( 0, 0, d );
    boost::posix_time::ptime t = boost::posix_time::from_iso_string( "20101010T121212.123456" );
    double error = 0.1;
    double derivative = 0;
    EXPECT_EQ( d*derivative, pid.update( error, t ) );
    int dt = 1;
    t += boost::posix_time::seconds( dt );
    double previous_error = error;
    error = 0.11;
    derivative = ( error - previous_error )/ dt;
    EXPECT_EQ( d*derivative, pid.update( error, t ) );
    dt = 2;
    t += boost::posix_time::seconds( dt );
    previous_error = error;
    error = 0.12;
    derivative = ( error - previous_error )/ dt;
    EXPECT_EQ( d*derivative, pid.update( error, t ) );
}

} } // namespace snark {  namespace test {

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
