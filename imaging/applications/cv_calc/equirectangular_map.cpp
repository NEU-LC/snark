// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2019 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

/// @author vsevolod vlaskine

#include <iostream>
#include <sstream>
#include <boost/static_assert.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <tbb/parallel_for.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/tuple/tuple.hpp>
#include <comma/base/exception.h>
#include <comma/csv/ascii.h>
#include <comma/visiting/traits.h>
#include "../../../visiting/eigen.h"
#include "equirectangular_map.h"

namespace snark { namespace cv_calc { namespace equirectangular_map {
    
static cv::Mat rotation_matrix( double x,double y,double z ) // quick and dirty
{
    cv::Mat r_x = ( cv::Mat_<double>(3,3) <<
        1,            0,            0,
        0,  std::cos(x), -std::sin(x),
        0,  std::sin(x),  std::cos(x) );
    cv::Mat r_y = ( cv::Mat_<double>(3,3) <<
        std::cos(y),    0,  std::sin(y),
                  0,    1,            0,
       -std::sin(y),    0,  std::cos(y) );
    cv::Mat r_z = ( cv::Mat_<double>(3,3) <<
        std::cos(z), -std::sin(z),      0,
        std::sin(z),  std::cos(z),      0,
                  0,            0,      1 );
    return r_z * r_y * r_x;
}
    
// see: inverse formula for spherical projection, Szeliski, "Computer Vision: Algorithms and Applications" p439.
struct calculator
{
    calculator( const Eigen::Vector3d& orientation, cv::Mat K, unsigned int w, unsigned int h )
        : RK( rotation_matrix( orientation.y(), orientation.z(), orientation.x() ) * K.inv() ) // todo: validate euler angle order
        , w( w )
        , h( h )
    {
        xyz = ( cv::Mat_< double >( 3, 1 ) << 0, 0, 1 );
    }

    std::pair< double, double > projected_pixel( unsigned int x, unsigned int y )
    {
        xyz.at< double >( 0, 0 ) = x;
        xyz.at< double >( 1, 0 ) = y;
        cv::Mat ray3d = RK * ( xyz / cv::norm( xyz ) ); 
        double xp = ray3d.at< double >( 0, 0 );
        double yp = ray3d.at< double >( 0, 1 );
        double zp = ray3d.at< double >( 0, 2 );    
        double phi = std::atan2( xp, zp );
        double sx = ( phi / ( M_PI * 2 ) + 0.5 ) * w;
        if( sx >= w - 0.00001 ) { sx = 0; }
        double theta = std::atan2( yp, std::sqrt( xp * xp + zp * zp ) );
        double sy = ( theta / M_PI + 0.5 ) * h;
        return std::make_pair( sx, sy );
    };
    
    cv::Mat RK;
    cv::Mat xyz;
    unsigned int w;
    unsigned int h;
};

std::string options()
{
    std::ostringstream oss;
    oss << "        --focal-length=<pixels>; camera focal length" << std::endl;
    oss << "        --map-size,--size=<width>,<height>; output map size" << std::endl;
    oss << "        --orientation=<roll>,<pitch>,<yaw>; default=0,0,0; orientation in radians" << std::endl;
    oss << "        --spherical-size=<width>,<height>; input spherical size" << std::endl;
    oss << "        --cubes=<width>; output map for top,back,left,front,right,bottom cubes with a given original orientation" << std::endl;
    oss << std::endl;
    return oss.str();
}

int run( const comma::command_line_options& options )
{
    BOOST_STATIC_ASSERT( sizeof( float ) == 4 );
    auto focal_length = options.optional< double >( "--focal-length" );
    unsigned int spherical_width, spherical_height, map_width, map_height;
    bool cubes = options.exists( "--cubes" );
    if( cubes ) { map_width = map_height = options.value< unsigned int >( "--cubes" ); }
    else { boost::tie( map_width, map_height ) = comma::csv::ascii< std::pair< unsigned int, unsigned int > >().get( options.value< std::string >( "--map-size,--size" ) ); }
    boost::tie( spherical_width, spherical_height ) = comma::csv::ascii< std::pair< unsigned int, unsigned int > >().get( options.value< std::string >( "--spherical-size" ) );
    if( !focal_length )
    {
        if( map_width != map_height ) { std::cerr << "cv-calc: equirectangular-map: please specify --focal-length" << std::endl; return 1; }
        focal_length = map_width / 2;
    }
    cv::Mat camera = ( cv::Mat_< double >( 3, 3 ) << *focal_length,             0,  map_width / 2,
                                                                    0, *focal_length, map_height / 2,
                                                                    0,             0,              1 );
    auto orientation = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--orientation", "0,0,0" ) );
    auto make_map = [&]( const Eigen::Vector3d& o ) -> std::pair< cv::Mat, cv::Mat >
    {
        cv::Mat x( map_height, map_width, CV_32F ); // quick and dirty; if output to stdout has problems, use serialisation class
        cv::Mat y( map_height, map_width, CV_32F ); // quick and dirty; if output to stdout has problems, use serialisation class
        tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, map_height ), [&]( const tbb::blocked_range< std::size_t >& r )
        {
            calculator calc( o, camera, spherical_width, spherical_height );
            for( unsigned int v = r.begin(); v < r.end(); ++v )
            {
                for( unsigned int u = 0; u < map_width; ++u )
                {
                    boost::tie( x.at< float >( v, u ), y.at< float >( v, u ) ) = calc.projected_pixel( u, v );
                }
            }
        } );
        return std::make_pair( x, y );
    };
    auto face = make_map( orientation );
    if( !cubes )
    {
        std::cout.write( reinterpret_cast< const char* >( face.first.datastart ), face.first.dataend - face.first.datastart );
        std::cout.write( reinterpret_cast< const char* >( face.second.datastart ), face.second.dataend - face.second.datastart );
        std::cout.flush();
        return 0;
    }
    if( orientation != Eigen::Vector3d::Zero() ) { std::cerr << "cv-calc: equirectangular-map: for cubes, expected --orientation 0,0,0; got: " << options.value< std::string >( "--orientation" ) << " (not supported)" << std::endl; return 1; }
    auto top = make_map( Eigen::Vector3d( 0, M_PI / 2, orientation.z() ) );
    auto bottom = make_map( Eigen::Vector3d( 0, -M_PI / 2, orientation.z() ) );
    std::cout.write( reinterpret_cast< const char* >( top.first.datastart ), top.first.dataend - top.first.datastart );
    int width = spherical_width / 4;
    for( int i = -2; i < 2; ++i )
    {
        cv::Mat f = face.first + width * i; // todo? waste to allocate it each time (currently, the only reason for it is the back face
        if( i == -2 ) { cv::Mat( f, cv::Rect( 0, 0, f.cols / 2, f.rows ) ) += spherical_width; }
        std::cout.write( reinterpret_cast< const char* >( f.datastart ), f.dataend - f.datastart );
    }
    std::cout.write( reinterpret_cast< const char* >( bottom.first.datastart ), bottom.first.dataend - bottom.first.datastart );
    std::cout.write( reinterpret_cast< const char* >( top.second.datastart ), top.second.dataend - top.second.datastart );
    for( unsigned int i = 0; i < 4; ++i ) { std::cout.write( reinterpret_cast< const char* >( face.second.datastart ), face.second.dataend - face.second.datastart ); }
    std::cout.write( reinterpret_cast< const char* >( bottom.second.datastart ), bottom.second.dataend - bottom.second.datastart );
    std::cout.flush();
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace equirectangular_map {
