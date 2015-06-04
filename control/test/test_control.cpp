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

TEST( pid, constructor )
{
    {
        snark::control::pid<> p( 0.1, 0.01, 0.001 );
        snark::control::pid< snark::control::internal > p1( 0.1, 0.01, 0.001 );
        snark::control::pid< snark::control::internal > p2( 0.1, 0.01, 0.001, 0.25 );
        snark::control::pid< snark::control::external > p3( 0.1, 0.01, 0.001 );
        snark::control::pid< snark::control::external > p4( 0.1, 0.01, 0.001, 0.25 );
    }
    {
        snark::control::pid< snark::control::internal > p1( "0.1,0.01,0.001" );
        snark::control::pid< snark::control::external > p2( "0.1,0.01,0.001" );
        snark::control::pid< snark::control::external > p3( "0.1;0.01;0.001", ';' );
        snark::control::pid< snark::control::external > p4( "0.1,0.01,0.001,0.25" );
        snark::control::pid< snark::control::external > p5( "0.1;0.01;0.001;0.25", ';' );
    }
}

TEST( pid, threshold_throw )
{
    ASSERT_THROW( snark::control::pid<>( 0, 0, 0, -1.0 ), comma::exception );
    ASSERT_THROW( snark::control::pid<>( 0, 0, 0, 0.0 ), comma::exception );
    ASSERT_THROW( snark::control::pid<>( "0,0,0,-1.0" ), comma::exception );
    ASSERT_THROW( snark::control::pid<>( "0,0,0,0.0" ), comma::exception );
}

TEST( pid, parsing_throw )
{
    ASSERT_THROW( snark::control::pid<>( "0,0" ), comma::exception );
    ASSERT_THROW( snark::control::pid<>( "0,0,0,-1,0" ), comma::exception );
}

static const boost::posix_time::ptime initial_time = boost::posix_time::from_iso_string( "20101010T121212.123456" );

TEST( pid, zero_time_increment_throw )
{
    snark::control::pid<> pid( 0, 0, 0 );
    boost::posix_time::ptime t = initial_time;
    pid.update( 0, t );
    ASSERT_THROW( pid.update( 0, t ), comma::exception );
}

TEST( pid, proportional )
{
    const double p = 0.12;
    snark::control::pid<> pid( p, 0, 0 );
    boost::posix_time::ptime t = initial_time;
    double error = 0.1;
    EXPECT_EQ( p*error, pid.update( error, t ) );
    t += boost::posix_time::microseconds( 1 );
    error = 0.11;
    EXPECT_EQ( p*error, pid.update( error, t ) );
    t += boost::posix_time::microseconds( 2 );
    error = 0.12;
    EXPECT_EQ( p*error, pid.update( error, t ) );
}

double to_seconds( double microseconds ) { return microseconds/1e6; }

TEST( pid, integral )
{
    const double i = 0.04;
    snark::control::pid<> pid( 0, i, 0 );
    boost::posix_time::ptime t = initial_time;
    double error = 0.1;
    double integral = 0;
    EXPECT_EQ( i*integral, pid.update( error, t ) );
    int microseconds = 10;
    t += boost::posix_time::microseconds( microseconds );
    error = 0.11;
    integral += error * to_seconds( microseconds );
    EXPECT_EQ( i*integral, pid.update( error, t ) );
    microseconds = 20;
    t += boost::posix_time::microseconds( microseconds );
    error = -0.12;
    integral += error * to_seconds( microseconds );
    EXPECT_EQ( i*integral, pid.update( error, t ) );
}

TEST( pid, threshold )
{
    const double i = 0.1;
    const double threshold = 0.00015;
    snark::control::pid<> pid( 0, i, 0, threshold );
    boost::posix_time::ptime t = initial_time;
    double error = 0;
    EXPECT_EQ( 0, pid.update( error, t ) );
    int microseconds = 2000;
    t += boost::posix_time::microseconds( microseconds );
    error = -0.1;
    EXPECT_EQ( i*(-threshold), pid.update( error, t ) );
}

TEST( pid, derivative_internal )
{
    const double d = 0.1;
    snark::control::pid<> pid( 0, 0, d );
    boost::posix_time::ptime t = initial_time;
    double error = 0.1;
    EXPECT_EQ( 0, pid.update( error, t ) );
    int microseconds = 1;
    t += boost::posix_time::microseconds( microseconds );
    double previous_error = error;
    error = 0.11;
    double derivative = ( error - previous_error )/ to_seconds( microseconds );
    EXPECT_EQ( d*derivative, pid.update( error, t ) );
    microseconds = 2;
    t += boost::posix_time::microseconds( microseconds );
    previous_error = error;
    error = 0.12;
    derivative = ( error - previous_error )/ to_seconds( microseconds );
    EXPECT_EQ( d*derivative, pid.update( error, t ) );
}

TEST( pid, derivative_external )
{
    const double d = 0.1;
    snark::control::pid< snark::control::external > pid( 0, 0, d );
    boost::posix_time::ptime t = initial_time;
    double error = 0;
    EXPECT_EQ( 0, pid.update( error, t ) );
    t += boost::posix_time::microseconds( 1 );
    double derivative = 0.1;
    EXPECT_EQ( d*derivative, pid.update( error, t, derivative ) );
    t += boost::posix_time::microseconds( 2 );
    derivative = 0.2;
    EXPECT_EQ( d*derivative, pid.update( error, t, derivative ) );
}

TEST( pid, reset )
{
    double i = 0.1;
    snark::control::pid<> pid( 0, i, 0 );
    boost::posix_time::ptime t = initial_time;
    double error = 0.1;
    double integral = 0;
    EXPECT_EQ( i*integral, pid.update( error, t ) );
    int microseconds = 2;
    t += boost::posix_time::microseconds( microseconds );
    error = 0.2;
    integral += error * to_seconds( microseconds );
    EXPECT_EQ( i*integral, pid.update( error, t ) );
    pid.reset();
    integral = 0;
    EXPECT_EQ( i*integral, pid.update( error, t ) );
    microseconds = 3;
    t += boost::posix_time::microseconds( microseconds );
    error = 0.3;
    integral = error * to_seconds( microseconds );
    EXPECT_EQ( i*integral, pid.update( error, t ) );
}

TEST( pid, combination )
{
    double p = 0.1;
    double i = 0.01;
    double d = 0.001;
    snark::control::pid<> pid( p, i, d );
    boost::posix_time::ptime t = initial_time;
    double error = 0.1;
    double integral = 0;
    double derivative = 0;
    EXPECT_EQ( p*error + i*integral + d*derivative, pid.update( error, t ) );
    int microseconds = 2;
    t += boost::posix_time::microseconds( microseconds );
    double previous_error = error;
    error = -0.2;
    integral += error * to_seconds( microseconds );
    derivative = ( error - previous_error ) / to_seconds( microseconds );
    EXPECT_EQ( p*error + i*integral + d*derivative, pid.update( error, t ) );
    microseconds = 3;
    t += boost::posix_time::microseconds( microseconds );
    previous_error = error;
    error = 0.3;
    integral += error * to_seconds( microseconds );
    derivative = ( error - previous_error ) / to_seconds( microseconds );
    EXPECT_EQ( p*error + i*integral + d*derivative, pid.update( error, t ) );
}

} } // namespace snark {  namespace test {

int main( int ac, char** av )
{
    ::testing::InitGoogleTest( &ac, av );
    return RUN_ALL_TESTS();
}
