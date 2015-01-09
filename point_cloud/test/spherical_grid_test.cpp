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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the University of Sydney.
// 4. Neither the name of the University of Sydney nor the
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
#include <snark/point_cloud/spherical_grid.h>

namespace snark {

TEST( spherical_grid, bearing_elevation_index_usage_example )
{
    bearing_elevation begin( -M_PI, -M_PI / 2 );
    bearing_elevation resolution( 0.1, 0.02 );
    bearing_elevation_grid::index index( begin, resolution );
    bearing_elevation_grid::index::type size = index( bearing_elevation( 1.2, 0.6 ) );
    boost::multi_array< std::string, 2 > grid( boost::extents[size[0]][size[1]] );
    bearing_elevation_grid::index::type some_point = index( bearing_elevation( 1.1, 0.5 ) );
    grid( some_point ) = "hello world";
}

static const double one_degree = M_PI / 180;

TEST( spherical_grid, bearing_elevation_gid_usage_example )
{
    bearing_elevation_grid::type< std::string > grid( bearing_elevation_grid::index( one_degree ), 360, 180 );
    grid( 20 * one_degree, 30 * one_degree ) = "hello world";
    EXPECT_EQ( "hello world", grid( 20 * one_degree, 30 * one_degree ) );
    EXPECT_EQ( "", grid( 19.5 * one_degree, 30 * one_degree ) );
}

TEST( spherical_grid, bearing_elevation_grid_bearing_index )
{
    for( int i = -180; i < 180; ++i )
    {
        EXPECT_EQ( i + 180, bearing_elevation_grid::index( one_degree )( one_degree * i, 0 )[0] );
        EXPECT_EQ( i + 180, bearing_elevation_grid::index( one_degree )( one_degree * ( 0.5 + i ), 0 )[0] );
    }
    // todo: test with arbitrary begin
}

TEST( spherical_grid, bearing_elevation_grid_elevation_index )
{
    for( int i = -90; i < 90; ++i )
    {
        EXPECT_EQ( i + 90, bearing_elevation_grid::index( one_degree )( 0, one_degree * i )[1] );
        EXPECT_EQ( i + 90, bearing_elevation_grid::index( one_degree )( 0, one_degree * ( 0.5 + i ) )[1] );
    }
    // todo: test with arbitrary begin
}

TEST( bearing_index, bearing_wraparound_mapping )
{
    const double step = 1.25 * one_degree;
    const double epsilon = 0.000001;
    bearing_elevation_grid::bearing_index index( step );
    unsigned int size = 360 / 1.25;
    double angle = -M_PI;
    for( unsigned int i = 0; i < size; ++i, angle += step ) { EXPECT_EQ( i, index( angle ) ); }

    EXPECT_EQ( 0, index( -M_PI ) );
    EXPECT_EQ( 0, index( -M_PI + epsilon ) );
    EXPECT_EQ( 1, index( -M_PI + step ) );
    EXPECT_EQ( 143, index( 0 - epsilon ) );
    EXPECT_EQ( 144, index( 0 ) );
    EXPECT_EQ( 144, index( 0 + epsilon ) );
    EXPECT_EQ( 287, index( M_PI - step ) );
    EXPECT_EQ( 287, index( M_PI - epsilon ) );
    EXPECT_EQ( 0, index( M_PI ) );
    EXPECT_EQ( 0, index( M_PI + epsilon ) );
    EXPECT_EQ( 1, index( M_PI + step ) ); /// Wrapping around
    EXPECT_EQ( 144, index( 2 * M_PI) );
    EXPECT_EQ( 0, index( 3 * M_PI) );
    EXPECT_EQ( 144, index( 4 * M_PI) );
    EXPECT_EQ( 0, index( 5 * M_PI) );

}

TEST( elevation_index, elevation_wraparound_mapping )
{
    double step = 1.25 * one_degree;
    bearing_elevation_grid::elevation_index index( step );
    unsigned int i = 0;
    for( double angle = -M_PI / 2; angle < M_PI / 2; ++i, angle += step ) { EXPECT_EQ( i, index( angle ) ); }

    EXPECT_EQ( 1, index( -M_PI/2 - step ) );
    EXPECT_EQ( 0, index( -M_PI/2 ) );
    EXPECT_EQ( 1, index( -M_PI/2 + step ) );

    EXPECT_EQ( 72, index( 0 ) );

    EXPECT_EQ( 143, index( M_PI/2 - step ) );
    EXPECT_EQ( 144, index( M_PI/2 ) );
    EXPECT_EQ( 143, index( M_PI/2 + step ) ); /// Wrapping around
    EXPECT_EQ( 142, index( M_PI/2 + 2*step ) );

}

TEST( bearing_index, get_bounds )
{
    const double epsilon = 1e-6;
    {
        double step = 1.25 * one_degree;
        bearing_elevation_grid::bearing_index index(-M_PI, step );
        bearing_elevation_grid::bounds b;
        unsigned int i = 0;

        /// First, check that the range [-PI, PI) correctly
        /// maps to bearing_index values 0 to 287
        for( double angle = -M_PI ; angle < M_PI ; angle = -M_PI + step * ++i )
        {
            b = index.get_bounds(angle);
            EXPECT_EQ( i, b.lower_index);
            EXPECT_EQ( i, b.upper_index);
            EXPECT_NEAR( 0, b.scaled_distance, epsilon);
        }

        /// Verify that longitudes equivalent to -PI
        /// have a lower index and upper index of 0.
       for( double angle = -M_PI * 5 ; angle <= M_PI * 5; angle += M_PI * 2 )
        {
            b = index.get_bounds(angle);
            EXPECT_EQ( 0, b.lower_index);
            EXPECT_EQ( 0, b.upper_index);
            EXPECT_NEAR( 0, b.scaled_distance, epsilon);
        }

        /// Border conditions around M_PI follow
        b = index.get_bounds( M_PI - step);
        EXPECT_EQ( 287, b.lower_index);
        EXPECT_EQ( 287, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        b = index.get_bounds( M_PI + step);
        EXPECT_EQ( 1, b.lower_index);
        EXPECT_EQ( 1, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        b = index.get_bounds( M_PI - epsilon);
        EXPECT_EQ( 287, b.lower_index);
        EXPECT_EQ( 0, b.upper_index);
        EXPECT_NEAR( 0.999954, b.scaled_distance, epsilon);

        b = index.get_bounds( M_PI + epsilon);
        EXPECT_EQ( 0, b.lower_index);
        EXPECT_EQ( 1, b.upper_index);
        EXPECT_NEAR( 0.000046, b.scaled_distance, epsilon);

        b = index.get_bounds( M_PI + step/2);
        EXPECT_EQ( 0, b.lower_index);
        EXPECT_EQ( 1, b.upper_index);
        EXPECT_NEAR( 0.5, b.scaled_distance, epsilon);

        b = index.get_bounds( M_PI - step/2);
        EXPECT_EQ( 287, b.lower_index);
        EXPECT_EQ( 0, b.upper_index);
        EXPECT_NEAR( 0.5, b.scaled_distance, epsilon);


        /// Border conditions around -M_PI follow
        b = index.get_bounds( -M_PI - step);
        EXPECT_EQ( 287, b.lower_index);
        EXPECT_EQ( 287, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        b = index.get_bounds( -M_PI + step);
        EXPECT_EQ( 1, b.lower_index);
        EXPECT_EQ( 1, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        b = index.get_bounds( -M_PI - epsilon);
        EXPECT_EQ( 287, b.lower_index);
        EXPECT_EQ( 0, b.upper_index);
        EXPECT_NEAR( 0.999954, b.scaled_distance, epsilon);

        b = index.get_bounds( -M_PI + epsilon);
        EXPECT_EQ( 0, b.lower_index);
        EXPECT_EQ( 1, b.upper_index);
        EXPECT_NEAR( 0.000046, b.scaled_distance, epsilon);
    }

    /// Test bearing_index using a step of one degree rather than 1.25 degree
    {
        bearing_elevation_grid::bearing_index index(-M_PI, one_degree );
        bearing_elevation_grid::bounds b = index.get_bounds( -M_PI - 0.00001 );
        EXPECT_EQ( 359, b.lower_index );
        EXPECT_EQ( 0, b.upper_index );
        EXPECT_NEAR( ( one_degree - 0.00001 ) / one_degree, b.scaled_distance, epsilon );
    }

    /// Test bearing_index that has a begin value of 0 rather than -PI
    {
        double step = 1.25 * one_degree;
        bearing_elevation_grid::bearing_index index(0, step );
        bearing_elevation_grid::bounds b;
        unsigned int i = 0;

        for( double angle = 0 ; angle < M_PI * 2 ; angle = 0 + step * ++i )
        {
            b = index.get_bounds(angle);
            EXPECT_EQ( i, b.lower_index);
            EXPECT_EQ( i, b.upper_index);
            EXPECT_NEAR( 0, b.scaled_distance, epsilon);
        }
    }

    /// Test bearing_index that has a begin value of PI/2 rather than -PI
    {
        double step = 1.25 * one_degree;
        bearing_elevation_grid::bearing_index index(M_PI/2, step );
        bearing_elevation_grid::bounds b;
        unsigned int i = 0;

        for( double angle = M_PI/2 ; angle < M_PI * 2 +  M_PI/2; angle = M_PI/2 + step * ++i )
        {
            b = index.get_bounds(angle);
            EXPECT_EQ( i, b.lower_index);
            EXPECT_EQ( i, b.upper_index);
            EXPECT_NEAR( 0, b.scaled_distance, epsilon);
        }
    }
}


TEST( elevation_index, get_bounds )
{
    const double epsilon = 1e-6;
    {
        double step_degrees = 1.25;
        double step = step_degrees * one_degree;
        double begin = -M_PI/2;
        bearing_elevation_grid::elevation_index index(begin, step );
        bearing_elevation_grid::bounds b;
        unsigned int i = 0;

        /// First, check that the range [-PI/2, PI/2] correctly
        /// maps to elevation_index values [0, 144]
        for( double angle = -M_PI / 2; angle <= M_PI / 2; angle = -M_PI/2 + step * ++i )
        {
            b = index.get_bounds(angle);
            EXPECT_EQ( i, b.lower_index);
            EXPECT_EQ( i, b.upper_index);
            EXPECT_NEAR( 0, b.scaled_distance, epsilon);
        }

        /// Latitudes equivalent to 0 follow

        b = index.get_bounds( 0.0 );
        EXPECT_EQ( 72, b.lower_index);
        EXPECT_EQ( 72, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        b = index.get_bounds( M_PI );
        EXPECT_EQ( 72, b.lower_index);
        EXPECT_EQ( 72, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        b = index.get_bounds( -M_PI );
        EXPECT_EQ( 72, b.lower_index);
        EXPECT_EQ( 72, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        b = index.get_bounds( M_PI * 2 );
        EXPECT_EQ( 72, b.lower_index);
        EXPECT_EQ( 72, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        b = index.get_bounds( 0.0 + epsilon );
        EXPECT_EQ( 72, b.lower_index);
        EXPECT_EQ( 73, b.upper_index);
        EXPECT_NEAR( 0.000046, b.scaled_distance, epsilon);

        b = index.get_bounds( M_PI - epsilon );
        EXPECT_EQ( 72, b.lower_index);
        EXPECT_EQ( 73, b.upper_index);
        EXPECT_NEAR( 0.000046, b.scaled_distance, epsilon);

        /// Border conditions around M_PI/2 follow

        b = index.get_bounds( M_PI/2 );
        EXPECT_EQ( 144, b.lower_index);
        EXPECT_EQ( 144, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        b = index.get_bounds( M_PI/2 - epsilon);
        EXPECT_EQ( 143, b.lower_index);
        EXPECT_EQ( 144, b.upper_index);
        EXPECT_NEAR( 0.999954, b.scaled_distance, epsilon);

        b = index.get_bounds( M_PI/2 + epsilon);
        EXPECT_EQ( 143, b.lower_index);
        EXPECT_EQ( 144, b.upper_index);
        EXPECT_NEAR( 0.999954, b.scaled_distance, epsilon);

        b = index.get_bounds( M_PI/2 - step);
        EXPECT_EQ( 143, b.lower_index);
        EXPECT_EQ( 143, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        b = index.get_bounds( M_PI/2 + step);
        EXPECT_EQ( 143, b.lower_index);
        EXPECT_EQ( 143, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        /// Border conditions around -M_PI/2 follow

        b = index.get_bounds( -M_PI/2 );
        EXPECT_EQ( 0, b.lower_index);
        EXPECT_EQ( 0, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        b = index.get_bounds( -M_PI/2 - epsilon);
        EXPECT_EQ( 0, b.lower_index);
        EXPECT_EQ( 1, b.upper_index);
        EXPECT_NEAR( 0.000046, b.scaled_distance, epsilon);

        b = index.get_bounds( -M_PI/2 + epsilon);
        EXPECT_EQ( 0, b.lower_index);
        EXPECT_EQ( 1, b.upper_index);
        EXPECT_NEAR( 0.000046, b.scaled_distance, epsilon);

        b = index.get_bounds( -M_PI/2 - step);
        EXPECT_EQ( 1, b.lower_index);
        EXPECT_EQ( 1, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        b = index.get_bounds( -M_PI/2 + step);
        EXPECT_EQ( 1, b.lower_index);
        EXPECT_EQ( 1, b.upper_index);
        EXPECT_NEAR( 0, b.scaled_distance, epsilon);

        b = index.get_bounds( -M_PI/2 + step/2);
        EXPECT_EQ( 0, b.lower_index);
        EXPECT_EQ( 1, b.upper_index);
        EXPECT_NEAR( 0.5, b.scaled_distance, epsilon);

        b = index.get_bounds( -M_PI/2 + step/2 - epsilon);
        EXPECT_EQ( 0, b.lower_index);
        EXPECT_EQ( 1, b.upper_index);
        EXPECT_NEAR( 0.499954, b.scaled_distance, epsilon);

        b = index.get_bounds( -M_PI/2 + step/2 + epsilon);
        EXPECT_EQ( 0, b.lower_index);
        EXPECT_EQ( 1, b.upper_index);
        EXPECT_NEAR( 0.500045, b.scaled_distance, epsilon);

        b = index.get_bounds( -M_PI/2 + step/4);
        EXPECT_EQ( 0, b.lower_index);
        EXPECT_EQ( 1, b.upper_index);
        EXPECT_NEAR( 0.25, b.scaled_distance, epsilon);
    }
}

} // namespace snark {
