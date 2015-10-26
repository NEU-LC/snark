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
#include "../spherical_geometry/ellipsoid.h"
#include <cstdlib>
#include <iostream>

using namespace snark::spherical;

namespace snark
{
namespace math
{


const double pi = std::atan( 1.0 ) * 4;

//1 cm
const double precision = 1.01e-5;


//input x degree; output radian
double inline rad( double x )
{
    return ( x * pi ) / 180.0;
}

double inline degree( double rd )
{
    return ( rd * 180 ) / pi;
}

//input degree; output coordiantes in radian
coordinates inline pos( double x, double y )
{
    return coordinates( rad( x ), rad( y ) );
}

TEST( ellipsoid, distance )
{

    //distance in km, angle in degree
    ellipsoid e( 6378.1370, 6356.7523 );
    EXPECT_EQ( 1, 3 - 2 );

    //close points and overlap
    //distance of 2 milli degree longitudinal at pole
    {
        const double tiny_polar_distance = 0.223388;
        EXPECT_NEAR( e.distance( pos( 89.999 , 0 ), pos( 89.999, 180 ) ), tiny_polar_distance,  precision );
        EXPECT_NEAR( e.distance( pos( -89.999 , 0 ), pos( -89.999, 180 ) ), tiny_polar_distance,  precision );
        for ( double i = 0; i < 100; i += 0.5 )
            EXPECT_NEAR( e.distance( pos( 89.999 , i ), pos( 89.999, i + 180 ) ), tiny_polar_distance,  precision );
        EXPECT_NEAR( e.distance( pos( -89.999 , 180 ), pos( -89.999, 360 ) ), tiny_polar_distance,  precision );
        EXPECT_NEAR( e.distance( pos( -89.999 , -180 ), pos( -89.999, 0 ) ), tiny_polar_distance,  precision );
    }

    //distance of 2 milli degree longitudinal at equator
    {
        const double tiny_equator_distance = 0.222639;
        EXPECT_NEAR( e.distance( pos( 0 , 0.001 ), pos( 0, -0.001 ) ), tiny_equator_distance, precision );
        EXPECT_NEAR( e.distance( pos( 0 , 179.999 ), pos( 0, -179.999 ) ), tiny_equator_distance, precision );
        EXPECT_NEAR( e.distance( pos( 0 , 180.001 ), pos( 0, -180.001 ) ), tiny_equator_distance, precision );
        EXPECT_NEAR( e.distance( pos( 0 , 359.999 ), pos( 0, 360.001 ) ), tiny_equator_distance, precision );
        for ( double i = 0.1; i < 360; i += 0.5 )
        {
            EXPECT_NEAR( e.distance( pos( 0 , i + 0.001 ), pos( 0, i - 0.001 ) ), tiny_equator_distance, precision );
        }
        //...
    }

    EXPECT_NEAR( e.distance( pos( 45 , -179.9 ), pos( 45, 179.9 ) ), 15.769363 , precision );
    EXPECT_NEAR( e.distance( pos( 45 , 179.9 ), pos( 45, -179.9 ) ), 15.769363 , precision );
    //...

    //meridian
    {
        const double meridian_length = 20003.931459;
        //20003931.458
        EXPECT_NEAR( e.distance( pos( 90 , 12 ), pos( -90, 34 ) ),  meridian_length, precision * 2.5 );
        EXPECT_NEAR( e.distance( pos( 90 , 0 ), pos( -90, 0 ) ),  meridian_length, precision * 2.5 );
        EXPECT_NEAR( e.distance( pos( 90 , 0.001 ), pos( -90, -0.001 ) ),  meridian_length, precision * 2.5 );

        //lookup
        EXPECT_NEAR( e.distance( pos( 0 , 180 ), pos( 10, 180 ) ), 1105.854833, precision );
        EXPECT_NEAR( e.distance( pos( 0 , -180 ), pos( 10, 180 ) ), 1105.854833, precision );
        EXPECT_NEAR( e.distance( pos( 10 , 90 ), pos( 20, 90 ) ), 1106.511421, precision );
        EXPECT_NEAR( e.distance( pos( 10 , 90 ), pos( 20, 450 ) ), 1106.511421, precision );
    }


    //boundary zero distance
    {
        //pole
        EXPECT_NEAR( e.distance( pos( 90 , 12 ), pos( 90, 34 ) ), 0, .000001 );

        EXPECT_NEAR( e.distance( pos( 0 , 0 ), pos( 90, 0 ) ), 10001.965729, 0.00005 );

        EXPECT_NEAR( e.distance( pos( 0 , 0 ), pos( 0, 90 ) ), ( pi * 6378.137 ) / 2 , 0.00005 );
        EXPECT_NEAR( e.distance( pos( -90 , 12 ), pos( -90, 34 ) ), 0, precision );


        //
        EXPECT_NEAR( e.distance( pos( 0 , 180 ), pos( 0, -180 ) ), 0, precision );
        EXPECT_NEAR( e.distance( pos( 45 , 180 ), pos( 45, -180 ) ), 0, precision );
        EXPECT_NEAR( e.distance( pos( 10 , 180 ), pos( 10, -180 ) ), 0, precision );
    }

    // 0,0 to 0,180 will fail to converge

    //perth-syd YPPH (PER) – Pert
    coordinates Perth( rad( -31.9402777778 ), rad( 115.966944444 ) );
    //YSSY (SYD) – Sydney
    coordinates Sydney( rad( -33.9461111111 ), rad( 151.177222222 ) );
    //KJFK (JFK) – John F. Kennedy International Airport – New York, New York
    coordinates NewYork( rad( 40.63975 ), rad( -73.778925 ) );

    //lookup from online vincenty calculator
    {
        EXPECT_NEAR( e.distance( Perth, Sydney ), 3284.061441, 0.00005 );

        const double NYSdistance = 16012.848859;

        //crosses both equator and 180 meridian
        EXPECT_NEAR( e.distance( NewYork, Sydney ), NYSdistance, 0.001 );

        coordinates shift( rad( 0 ), rad( 120 ) );
        EXPECT_NEAR( e.distance( NewYork - shift, Sydney - shift ), NYSdistance, 0.001 );

        EXPECT_NEAR( e.distance( pos( 54.7670345234, -108.68594087 ), pos( 54.7666806519, -108.691352558 ) ), 0.350542, 0.001 );
    }

    //equal measure
    {
        //hemisphere symetry
        EXPECT_DOUBLE_EQ( e.distance( pos( 32, 0 ), pos( 32, 45 ) ), e.distance( pos( -32, 0 ), pos( -32, 45 ) ) );

        EXPECT_NEAR( e.distance( pos( 18, 22 ), pos( 23, 62 ) ), 4196.427624, 0.000001 );

        //lateral transform
        {
            const double a = 18;
            const double b = 23;
            const double d = 40;

            EXPECT_DOUBLE_EQ( e.distance( pos( a, 22 ), pos( b, 22 + d ) ), e.distance( pos( a, 85 ), pos( b, 85 + d ) ) );
        }

        //longitudinal arcs
        {
            for ( int i = 0; i < 10; i++ )
                EXPECT_DOUBLE_EQ( e.distance( pos( 25, 0 ), pos( 55, 0 ) ), e.distance( pos( 25, i * 10 ), pos( 55, i * 10 ) ) );
        }
    }

    //discretized arc
    {
        coordinates center( rad( 54.408625 ), rad( -110.2958583 ) );
        EXPECT_NEAR( e.distance( center, pos( 53.4247034463, -109.999058966 ) ), 111.236405, 0.000001 );
        EXPECT_NEAR( e.distance( center, pos( 54.5736364436, -108.598913562 ) ), 111.485153, 0.000001 );
        EXPECT_NEAR( e.distance( center, pos( 54.7670345234, -108.68594087 ) ), 111.460289, 0.000001 );

    }


}

TEST( ellipsoid, elapsed )
{
    //distance in km, angle in degree
    ellipsoid e( 6378.1370, 6356.7523 );
    coordinates p( rad( -31.9402777778 ), rad( 115.966944444 ) );
    coordinates s( rad( -33.9461111111 ), rad( 151.177222222 ) );
    coordinates delta( 0, rad( 1 ) );

    //do one million iterations; check test output to see how long does it take to run the test
    for ( int i = 0; i < 100000; i++ )
    {
        EXPECT_NEAR( e.distance( p, s ), 3284.061441, 0.00005 );
        p += delta;
        s += delta;
    }
}

//heading degree from north east is positive 90
double inline bearing( double heading )
{
    //bearing standard geometric radian; ie east=0,north=pi/2,south=-pi/2
    return rad( ( heading * -1 ) + 90 );
}

template<typename T>
static std::basic_ostream<T> &operator<<( std::basic_ostream<T> &s, const coordinates &c )
{
    return s << "(" << c.latitude << ", " << c.longitude << ")";
}

void hexagon_test( ellipsoid e, coordinates center, double radius )
{
    //std::cerr<<"hexagon_test; center:"<<center<<", radius:"<<radius<<std::endl;
    coordinates p[6];
    for ( int i = 0; i < 6; i++ )
        p[i] = e.at( center, radius, ( 2 * pi * i ) / 6 );
    //in a hexagon all sides are equal?? (not if length is comparable to ellipsoid's axis)
    double edge0 = e.distance( p[0], p[1] );
    for ( int i = 0; i < 5; i++ )
    {
        //std::cerr<<"hex test["<<i<<"]"<<std::endl;
        EXPECT_NEAR( e.distance( p[i], p[i + 1] ), edge0, precision );
    }
    EXPECT_NEAR( e.distance( center, e.at( center, radius, pi / 6 ) ), radius, precision );
}

TEST( ellipsoid, at )
{
    ellipsoid e( 6378.1370, 6356.7523 );
    //at vs distance consistency
    {
        for ( int i = 0; i < 10; i++ )
        {
            double distance = ( 1000.0 * std::rand() ) / RAND_MAX;
            coordinates c( ( pi * std::rand() ) / RAND_MAX, ( pi * std::rand() ) / RAND_MAX );
            coordinates p = e.at( c, distance, ( pi * std::rand() ) / RAND_MAX );
            EXPECT_NEAR( e.distance( c, p ), distance, precision );
        }
    }

    //hexagon_test
    {
        //center,radius
        hexagon_test( e, coordinates( 0, 0 ), 100 );
        hexagon_test( e, coordinates( .5, 1 ), 100 );
        hexagon_test( e, coordinates( pi / 2, 1 ), 100 );
        hexagon_test( e, coordinates( 0, pi ), 100 );
        //hexagon_test(e,coordinates(.8,0),5000);       //large radius fails

    }

    //discretized arc
    {
        coordinates center( rad( 54.408625 ), rad( -110.2958583 ) );
        double radius = 111.120; //60 (NM) * 1.85200
        coordinates p1 = e.at( center, radius, bearing( -79.80587459623894 ) );
        coordinates p2 = e.at( center, radius, bearing( 21.67151780871575 ) );
        coordinates start = pos( 53.42566404696991, -110.00002504319121 );
        coordinates end = pos( 54.766680651912395, -108.69135255811793 );

        EXPECT_NEAR( degree( p1.latitude ), degree( start.latitude ), precision );
        EXPECT_NEAR( degree( p1.longitude ), degree( start.longitude ), precision );
        EXPECT_NEAR( e.distance( p1, start ), 0, precision );

        EXPECT_NEAR( degree( p2.latitude ), degree( end.latitude ), precision );
        EXPECT_NEAR( degree( p2.longitude ), degree( end.longitude ), precision );
        EXPECT_NEAR( e.distance( p2, end ), 0, precision );

    }

}

TEST( ellipsoid, bearing )
{
    //sphere with 10,000 km equator to pole distance
    ellipsoid e( 6366.1978, 6366.1978 );
    //great_circle::arc northern(coordinates(0,0), coordinates(M_PI/4,0));
    //EXPECT_DOUBLE_EQ(northern.bearing(0),0);        //should be zero
    coordinates center( 0, 0 );
    double radius = 100;
    double angle = M_PI / 200;
    coordinates northern = e.at( center, radius, 0 );
    EXPECT_NEAR( northern.longitude, center.longitude, 0.001 );
    EXPECT_NEAR( northern.latitude, center.latitude + angle, 0.001 );

    coordinates eastern = e.at( center, radius, M_PI / 2 );
    EXPECT_NEAR( eastern.longitude, center.longitude + angle, 0.001 );
    EXPECT_NEAR( eastern.latitude, center.latitude, 0.001 );

    coordinates western = e.at( center, radius, -M_PI / 2 );
    EXPECT_NEAR( western.longitude, center.longitude - angle, 0.001 );
    EXPECT_NEAR( western.latitude, center.latitude, 0.001 );

    coordinates southern = e.at( center, radius, M_PI );
    EXPECT_NEAR( southern.longitude, center.longitude, 0.001 );
    EXPECT_NEAR( southern.latitude, center.latitude - angle, 0.001 );
}


}//math
}//snark
