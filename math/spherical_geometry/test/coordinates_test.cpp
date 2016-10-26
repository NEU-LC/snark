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
        coordinates::shortcut_latitude = coordinates::shortcut_longitude = coordinates::shortcut_none = 0;
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
        if ( verbose ) { std::cerr << "shortcut/latitude,shortcut/longitude,shortcut/none,time/near: " << coordinates::shortcut_latitude << "," << coordinates::shortcut_longitude << "," << coordinates::shortcut_none << "," << time_near << std::endl; }

        // if epsilon is very large, nearly all points are close
        coordinates::shortcut_latitude = coordinates::shortcut_longitude = coordinates::shortcut_none = 0;
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
        if ( verbose ) { std::cerr << "shortcut/latitude,shortcut/longitude,shortcut/none,time/far:  " << coordinates::shortcut_latitude << "," << coordinates::shortcut_longitude << "," << coordinates::shortcut_none << "," << time_far << std::endl; }

        return std::make_pair( time_near, time_far );
    }
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
