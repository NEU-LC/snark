// Copyright (c) 2013-2016. This code was produced by the
// Australian Centre for Field Robotics, The University of Sydney under
// the Future Flight Planning project, University Reference 13996, contract
// NSW-CPS-2011-015264, Work Orders 5, 7 and 8. The intellectual property
// ownership is as set out in these contracts, as registered with
// Commercial Development and Industry Partnerships.

#include <gtest/gtest.h>
#include <comma/math/compare.h>
#include <iomanip>

#include "../../../math/spherical_geometry/coordinates.h"
#include "../../../math/spherical_geometry/great_circle.h"

using namespace snark;
using namespace snark::spherical;

namespace {

    double distance( double a_latitude, double a_longitude, double b_latitude, double b_longitude )
    {
        return 2.0 * std::asin( std::sqrt( std::pow( std::sin( 0.5*( a_latitude - b_latitude ) ), 2 )
                                         + std::cos( a_latitude ) * std::cos( b_latitude ) * std::pow( std::sin( 0.5*( a_longitude - b_longitude ) ), 2 ) ) );
    }

    double distance( const coordinates & a, const coordinates & b )
    {
        return distance( a.latitude, a.longitude, b.latitude, b.longitude );
    }

    std::vector< double > setup_scales( double scale = 0.9, size_t nscale = 256 )
    {
        std::vector< double > rv;
        rv.reserve( nscale );
        for ( size_t i = 0; i < nscale; ++i ) { rv.push_back( std::pow( scale, i ) ); }
        return rv;
    }

    double minimal_distance( const coordinates & anchor, const std::vector< double > & scales, bool verbose = false )
    {
        double anchor_latitude = anchor.latitude;
        double anchor_longitude = anchor.longitude;
        if ( verbose ) { std::cerr << "--- around (" << std::setprecision(12) << anchor_latitude * 180 / M_PI << "," << anchor_longitude * 180 / M_PI << ":" << std::endl; }
        double max_relative_error = 0.0;
        for ( size_t i = 0; i < scales.size(); ++i )
        {
            double offset_latitude = anchor_latitude + scales[i] * M_PI / 180.0;
            double offset_longitude = anchor_longitude + scales[i] * M_PI / 180.0;
            coordinates offset( offset_latitude, offset_longitude );
            great_circle::arc a( anchor, offset );
            double snark_angle = a.angle();
            double expected_angle = distance( anchor, offset );
            double abs_error = std::abs( snark_angle - expected_angle );
            double rel_error = 2.0 * abs_error / ( snark_angle + expected_angle );
            double distance = expected_angle * 6371000;
            max_relative_error = std::max( rel_error, max_relative_error );
            if ( verbose ) {
                std::cerr << "scale,angle/arc,angle/exact,angle/error/relative,distance: "
                          << std::setprecision(12) << scales[i] << "\t" << snark_angle * 180.0 / M_PI << "\t" << expected_angle * 180.0 / M_PI << "\t" << rel_error << "\t" << distance << std::endl;
            }
        }
        return max_relative_error;
    }

    double at_the_end( const coordinates & anchor, const std::vector< double > & scales, bool verbose = false )
    {
        double anchor_latitude = anchor.latitude;
        double anchor_longitude = anchor.longitude;
        if ( verbose ) { std::cerr << "--- around (" << std::setprecision(12) << anchor_latitude * 180 / M_PI << "," << anchor_longitude * 180 / M_PI << ":" << std::endl; }
        double max_relative_error = 0.0;
        for ( size_t i = 0; i < scales.size(); ++i )
        {
            double offset_latitude = anchor_latitude + scales[i] * M_PI / 180.0;
            double offset_longitude = anchor_longitude + scales[i] * M_PI / 180.0;
            coordinates offset( offset_latitude, offset_longitude );
            great_circle::arc a( anchor, offset );
            double direct_length = distance( anchor, offset );
            double snark_length = a.angle();
            coordinates direct_finish = a.at( direct_length );
            coordinates snark_finish = a.at( snark_length );
            double direct_mismatch = distance( offset, direct_finish );
            double snark_mismatch = distance( offset, snark_finish );
            double rel_error = snark_mismatch / direct_length;
            max_relative_error = std::max( rel_error, max_relative_error );
            if ( verbose ) {
                std::cerr << "scale,distance,direct/mismatch,snark/mismatch,error/relative: "
                          << std::setprecision(12) << scales[i] << "\t" << direct_length << "\t" << direct_mismatch * 6371000 << "\t" << snark_mismatch * 6371000 << "\t" << rel_error << std::endl;
            }
        }
        return max_relative_error;
    }
}

TEST( great_circle, DISABLED_angle_01 )
{
    std::vector< double > scales = setup_scales();
    std::vector< coordinates > anchors;
    anchors.push_back( coordinates::from_degrees(  10 ,  20 ) );
    anchors.push_back( coordinates::from_degrees( -40,  -20 ) );
    anchors.push_back( coordinates::from_degrees(  70, -120 ) );
    anchors.push_back( coordinates::from_degrees( -30,  150 ) );
    anchors.push_back( coordinates::from_degrees(  50,   60 ) );
    anchors.push_back( coordinates::from_degrees(  80,   60 ) );
    anchors.push_back( coordinates::from_degrees(  85,   60 ) );
    anchors.push_back( coordinates::from_degrees( -89,  180 ) );
    for ( size_t i = 0; i < anchors.size(); ++i )
    {
        double near = minimal_distance( anchors[i], scales, false );
        EXPECT_LE( near, 1.0e-6 );
    }
}

TEST( great_circle, DISABLED_at_01 )
{
    std::vector< double > scales = setup_scales();
    std::vector< coordinates > anchors;
    anchors.push_back( coordinates::from_degrees(  10 ,  20 ) );
    anchors.push_back( coordinates::from_degrees( -40,  -20 ) );
    anchors.push_back( coordinates::from_degrees(  70, -120 ) );
    anchors.push_back( coordinates::from_degrees( -30,  150 ) );
    anchors.push_back( coordinates::from_degrees(  50,   60 ) );
    anchors.push_back( coordinates::from_degrees(  80,   60 ) );
    anchors.push_back( coordinates::from_degrees(  85,   60 ) );
    anchors.push_back( coordinates::from_degrees( -89,  180 ) );
    for ( size_t i = 0; i < anchors.size(); ++i )
    {
        double near = at_the_end( anchors[i], scales, false );
        EXPECT_LE( near, 1.0e-6 );
    }
}
