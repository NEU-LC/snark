// Copyright (c) 2013-2016. This code was produced by the
// Australian Centre for Field Robotics, The University of Sydney under
// the Future Flight Planning project, University Reference 13996, contract
// NSW-CPS-2011-015264, Work Orders 5, 7 and 8. The intellectual property
// ownership is as set out in these contracts, as registered with
// Commercial Development and Industry Partnerships.

#include <gtest/gtest.h>
#include <comma/math/compare.h>
#include <iomanip>
#include <cstdlib>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../../../math/spherical_geometry/coordinates.h"
#include "../../../math/spherical_geometry/great_circle.h"

using namespace snark;
using namespace snark::spherical;

namespace {
    std::pair< double, double > call_is_near( const coordinates & anchor, const std::vector< coordinates > & points, size_t repeat, double epsilon_near, double epsilon_far, bool verbose = false )
    {
        boost::posix_time::ptime start = boost::posix_time::microsec_clock::universal_time();
        {
            bool any_near = false;
            for ( size_t r = 0; r < repeat; ++r ) {
                for ( size_t i = 0; i < points.size(); ++i )
                {
                    any_near &= anchor.is_near( points[i], epsilon_near );
                }
            }
            EXPECT_FALSE( any_near );
        }
        double time_near = double( ( boost::posix_time::microsec_clock::universal_time() - start ).total_microseconds() ) / 1000000;

        // if epsilon is very large, nearly all points are close
        start = boost::posix_time::microsec_clock::universal_time();
        {
            bool any_far = false;
            for ( size_t r = 0; r < repeat; ++r ) {
                for ( size_t i = 0; i < points.size(); ++i )
                {
                    any_far &= ( !anchor.is_near( points[i], epsilon_far ) );
                }
            }
            EXPECT_FALSE( any_far );
        }
        double time_far = double( ( boost::posix_time::microsec_clock::universal_time() - start ).total_microseconds() ) / 1000000;

        return std::make_pair( time_near, time_far );
    }

    double deg2rad( double degrees ) { return degrees / 180.0 * M_PI; }
    double rad2deg( double radians ) { return radians * 180.0 / M_PI; }
}

TEST( coordinates, is_near_performance )
{
    bool verbose = false;
    std::vector< coordinates > points;
    size_t large = 1000;
    size_t repeat = 10;
    points.reserve( large * large );
    boost::posix_time::ptime start = boost::posix_time::microsec_clock::universal_time();
    for ( size_t i = 0; i < large; ++i )
    {
        double latitude = i * M_PI / 2 / large;
        for ( size_t j = 0; j < large; ++j )
        {
            double longitude = -M_PI / 2.0 + j * M_PI / large;
            points.push_back( coordinates( latitude, longitude ) );
        }
    }
    double generate_time = double( ( boost::posix_time::microsec_clock::universal_time() - start ).total_microseconds() ) / 1000000;
    if ( verbose ) { std::cerr << "time/generate: " << generate_time << std::endl; }

    {
        // in the Southern hemisphere, thus, never close; shall use shortcuts all the time unless the threshold is large
        coordinates anchor = coordinates::from_degrees( -10.0, 0.0 );
        std::pair< double, double > timings = call_is_near( anchor, points, repeat, 0.0001, M_PI, verbose );
        EXPECT_TRUE( timings.second >= 10 * timings.first );
    }

    {
        // make latitude the same for all points and the anchor; longitude is different so points are not really close
        coordinates anchor = coordinates::from_degrees( 10.0, 100.0 );
        for ( size_t i = 0; i < points.size(); ++i ) { points[i].latitude = anchor.latitude; }
        std::pair< double, double > timings = call_is_near( anchor, points, repeat, 0.0001, M_PI, verbose );
        EXPECT_TRUE( timings.second >= 10 * timings.first );
    }
}

TEST( coordinates, is_near_basic )
{
    EXPECT_TRUE(  coordinates::is_near( coordinates::from_degrees( 1.0, 1.0 ), coordinates::from_degrees( 1.0, 1.01 ), deg2rad(0.1)   ) );
    EXPECT_FALSE( coordinates::is_near( coordinates::from_degrees( 1.0, 1.0 ), coordinates::from_degrees( 1.0, 1.01 ), deg2rad(0.001) ) );
}

TEST( coordinates, is_near_across_equator )
{
    EXPECT_TRUE(  coordinates::is_near( coordinates::from_degrees( 0.1, 1.0 ), coordinates::from_degrees( -0.1, 1.0 ), deg2rad(0.25) ) );
    EXPECT_FALSE( coordinates::is_near( coordinates::from_degrees( 0.1, 1.0 ), coordinates::from_degrees( -0.1, 1.0 ), deg2rad(0.1)  ) );
}

TEST( coordinates, is_near_across_zero_meridian )
{
    EXPECT_TRUE(  coordinates::is_near( coordinates::from_degrees( 1.0,   0.1 ), coordinates::from_degrees( 1.0, -0.1 ), deg2rad(0.25) ) );
    EXPECT_FALSE( coordinates::is_near( coordinates::from_degrees( 1.0,   0.1 ), coordinates::from_degrees( 1.0, -0.1 ), deg2rad(0.1)  ) );
    EXPECT_TRUE(  coordinates::is_near( coordinates::from_degrees( 1.0, 359.9 ), coordinates::from_degrees( 1.0,  0.1 ), deg2rad(0.25) ) );
    EXPECT_FALSE( coordinates::is_near( coordinates::from_degrees( 1.0, 359.9 ), coordinates::from_degrees( 1.0,  0.1 ), deg2rad(0.1)  ) );
    EXPECT_TRUE(  coordinates::is_near( coordinates::from_degrees( 1.0, 360.0 ), coordinates::from_degrees( 1.0,  0.1 ), deg2rad(0.25) ) );
    EXPECT_FALSE( coordinates::is_near( coordinates::from_degrees( 1.0, 360.0 ), coordinates::from_degrees( 1.0,  0.1 ), deg2rad(0.09) ) );
}

TEST( coordinates, is_near_epsilon_above_globe )
{
    EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees( 10.0,  10.0 ), coordinates::from_degrees(  50.0, -20.0 ), deg2rad(180.1) ) );
    EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees( 60.0, 110.0 ), coordinates::from_degrees( -60.0, -70.0 ), deg2rad(180.1) ) );
    EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees( 90.0,   0.0 ), coordinates::from_degrees( -90.0, 180.0 ), deg2rad(180.1) ) );
}

TEST( coordinates, is_near_epsilon_half_globe )
{
    EXPECT_TRUE(  coordinates::is_near( coordinates::from_degrees( 10.0,  10.0 ), coordinates::from_degrees(  50.0, -20.0 ), deg2rad(179.999) ) );
    EXPECT_FALSE( coordinates::is_near( coordinates::from_degrees( 60.0, 110.0 ), coordinates::from_degrees( -60.0, -70.0 ), deg2rad(179.999) ) );
    EXPECT_FALSE( coordinates::is_near( coordinates::from_degrees( 90.0,   0.0 ), coordinates::from_degrees( -90.0, 180.0 ), deg2rad(179.999) ) );
}

TEST( coordinates, is_near_high_latitude_north )
{
    EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees( 85.0,  10.0 ), coordinates::from_degrees( 85.0, -20.0 ), deg2rad(5.0)  ) );
    EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees( 90.0, 110.0 ), coordinates::from_degrees( 85.0, -70.0 ), deg2rad(5.01) ) );
    EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees( 89.0,   0.0 ), coordinates::from_degrees( 89.0, 180.0 ), deg2rad(5.0)  ) );
}

TEST( coordinates, is_near_high_latitude_south )
{
    EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees( -85.0,  10.0 ), coordinates::from_degrees( -85.0, -20.0 ), deg2rad(5.0)  ) );
    EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees( -90.0, 110.0 ), coordinates::from_degrees( -85.0, -70.0 ), deg2rad(5.01) ) );
    EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees( -89.0,   0.0 ), coordinates::from_degrees( -89.0, 180.0 ), deg2rad(5.0)  ) );
}

TEST( coordinates, is_near_80_latitude )
{
    EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees(  80.1,  10.0 ), coordinates::from_degrees(  79.9,  10.0 ), deg2rad(0.5) ) );
    EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees( -80.1, 110.0 ), coordinates::from_degrees( -79.9, 110.1 ), deg2rad(0.5) ) );
    EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees( -80.1,  -0.1 ), coordinates::from_degrees( -79.9, 360.1 ), deg2rad(0.5) ) );
}

// NOTE: this test relies on the broken, numerically unstable implementation of angle calculations (third, precise branch of is_far)
// the test would start to fail once the problem is fixed; fix the test then, do not try to rid the implementation
TEST( coordinates, is_near_broken_precision )
{
    // enforce the 3rd branch of is_far that relies on unstable computations
    {
        double tiny_distance = rad2deg( 0.5 / 6371000.0 );   // 0.5 m at the Earth radius; still not zero
        double tiny_epsilon = deg2rad(0.5 * tiny_distance);
        EXPECT_TRUE( tiny_epsilon < coordinates::epsilon );
        // epsilon is smaller then distance, cannot be near; but angle is zero and is reported as near
        EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees( tiny_distance, 0.0 ), coordinates::from_degrees( 0.0, 0.0 ), tiny_epsilon ) );
        EXPECT_TRUE( coordinates::is_near( coordinates::from_degrees( 0.0, tiny_distance ), coordinates::from_degrees( 0.0, 0.0 ), tiny_epsilon ) );
    }
    // for reference, enforce NOT taking the 3rd branch of is_far that relies on unstable computations
    {
        double tiny_distance = rad2deg( 16.0 / 6371000.0 );   // 16 m at the Earth radius; still not zero
        double tiny_epsilon = deg2rad(0.5 * tiny_distance);
        EXPECT_FALSE( tiny_epsilon < coordinates::epsilon );
        // epsilon is smaller then distance, cannot be near; but angle is zero and is reported as near
        EXPECT_FALSE( coordinates::is_near( coordinates::from_degrees( tiny_distance, 0.0 ), coordinates::from_degrees( 0.0, 0.0 ), tiny_epsilon ) );
        EXPECT_FALSE( coordinates::is_near( coordinates::from_degrees( 0.0, tiny_distance ), coordinates::from_degrees( 0.0, 0.0 ), tiny_epsilon ) );
    }
}
