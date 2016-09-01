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

/// @authors vsevolod vlaskine, zhe xu

#include <boost/array.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/serialize.h>
#include "../camera/pinhole.h"
#include "../camera/traits.h"

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "take on stdin pixels, perform an operation based on pinhole camera model" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat pixels.csv | image-pinhole <operation> <options> > points.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    to-cartesian: take on stdin pixels, undistort image, append pixel's cartesian coordinates in camera frame" << std::endl;
    std::cerr << "        --normalize: normalize cartesian coordinates in camera frame" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to-pixels: take on stdin cartesian coordinates in camera frame, append their coordinates in pixels" << std::endl;
    std::cerr << "        --clip: clip pixels outside of image" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    undistort: take on stdin pixels, append their undistorted values" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    distort: take on stdin undistorted pixels, append their distorted values (uses distortion map file)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    distortion-map: build distortion map from camera parameters in config and write to stdout (binary image matrix of map x, map y)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: print help; --help --verbose for more help" << std::endl;
    std::cerr << "    --camera-config,--camera,--config,-c=<parameters>: camera configuration" << std::endl;
    std::cerr << "        <parameters>: filename of json or path-value configuration file or ';'-separated path-value pairs" << std::endl;
    std::cerr << "                      e.g: --config=\"focal_length=123;image_size/x=222;image_size/y=123.1;...\"" << std::endl;
    std::cerr << "    --input-fields: output input fields for given operation and exit" << std::endl;
    std::cerr << "    --output-config,--sample-config,--config-sample: output sample config and exit" << std::endl;
    std::cerr << "    --output-fields: output appended fields for given operation and exit" << std::endl;
    std::cerr << "    --output-format: output appended fields binary format for given operation and exit" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << std::endl;
    std:: cerr << snark::camera::pinhole::usage() << std::endl;
    if( verbose ) { std::cerr << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    config-to-normals:" << std::endl;
    std::cerr << "        todo" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

template < typename S, typename T > static void output_details( const comma::command_line_options& options )
{
    if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< S >(), ',' ) << std::endl; exit( 0 ); }
    if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< T >(), ',' ) << std::endl; exit( 0 ); }
    if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format::value< T >() << std::endl; exit( 0 ); }
}

static snark::camera::pinhole::config_t make_config( const std::string& config_parameters )
{
    snark::camera::pinhole::config_t config;
    if( config_parameters.find_first_of( '=' ) == std::string::npos )
    {
        const std::vector< std::string >& v = comma::split( config_parameters, ":#@" );
        if( v.size() == 1 ) { comma::read( config, config_parameters, true ); }
        else { comma::read( config, config_parameters.substr( 0, config_parameters.size() - v.back().size() - 1 ), v.back(), true ); }
        config.validate();
    }
    else
    {
        boost::property_tree::ptree p = comma::property_tree::from_path_value_string( config_parameters, '=', ';', comma::property_tree::path_value::no_check, true );
        comma::from_ptree from_ptree( p, true );
        comma::visiting::apply( from_ptree ).to( config );
    }
    return config;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--deprecated" ) ) { std::cerr << "image-pinhole: --deprecated support has been removed" << std::endl; return 1; }
        if( options.exists( "--output-config,--sample-config,--config-sample" ) )
        {
            snark::camera::pinhole::config_t config;
            config.focal_length = 0.1;
            config.image_size = Eigen::Vector2i( 1000, 2000 );
            config.sensor_size = Eigen::Vector2d( 0.1, 0.2 );
            config.distortion = snark::camera::pinhole::config_t::distortion_t( snark::camera::pinhole::config_t::distortion_t::radial_t( 0.001, -0.0002, 0.003 ), snark::camera::pinhole::config_t::distortion_t::tangential_t( 0.0004, -0.0005 ) );
            comma::write_json( config, std::cout );
            return 0;
        }
        const std::vector< std::string >& unnamed = options.unnamed( "--input-fields,--output-fields,--output-format,--verbose,-v,--flush,--clip,--keep,--normalize", "-.*" );
        if( unnamed.empty() ) { std::cerr << "image-pinhole: please specify operation" << std::endl; return 1; }
        std::string operation = unnamed[0];
        snark::camera::pinhole pinhole( make_config( options.value< std::string >( "--camera-config,--camera,--config,-c" ) ) );
        comma::csv::options csv( options );
        if( operation == "to-cartesian" )
        {
            output_details< Eigen::Vector2d, Eigen::Vector3d >( options );
            comma::csv::input_stream< Eigen::Vector2d > is( std::cin, csv );
            comma::csv::output_stream< Eigen::Vector3d > os( std::cout, csv.binary(), false, csv.flush );
            comma::csv::tied< Eigen::Vector2d, Eigen::Vector3d > tied( is, os );
            bool normalize = options.exists( "--normalize" );
            while( is.ready() || std::cin.good() )
            {
                const Eigen::Vector2d* p = is.read();
                if( !p ) { break; }
                const Eigen::Vector3d& q = pinhole.to_cartesian( *p );
                tied.append( normalize ? q.normalized() : q );
            }
            return 0;
        }
        if( operation == "to-pixels" )
        {
            output_details< Eigen::Vector2d, Eigen::Vector2d >( options );
            comma::csv::input_stream< Eigen::Vector2d > is( std::cin, csv );
            comma::csv::output_stream< Eigen::Vector2d > os( std::cout, csv.binary(), false, csv.flush );
            comma::csv::tied< Eigen::Vector2d, Eigen::Vector2d > tied( is, os );
            bool clip = options.exists( "--clip" ) || ( options.exists( "--deprecated" ) && !options.exists( "--keep" ) );
            while( is.ready() || std::cin.good() )
            {
                const Eigen::Vector2d* p = is.read();
                if( !p ) { break; }
                const Eigen::Vector2d& pixel = pinhole.to_pixel( *p );
                if ( !clip || (    !comma::math::less( pixel.x(), 0 )
                                && comma::math::less( pixel.x(), pinhole.config().image_size.x() )
                                && !comma::math::less( pixel.y(), 0 )
                                && comma::math::less( pixel.y(), pinhole.config().image_size.y() ) ) )
                {
                    tied.append( pinhole.distort( pixel ) );
                }
            }
            return 0;
        }
        if( operation == "undistort" )
        {
            output_details< Eigen::Vector2d, Eigen::Vector2d >( options );
            comma::csv::input_stream< Eigen::Vector2d > is( std::cin, csv );
            comma::csv::output_stream< Eigen::Vector2d > os( std::cout, csv.binary(), false, csv.flush );
            comma::csv::tied< Eigen::Vector2d, Eigen::Vector2d > tied( is, os );
            while( is.ready() || std::cin.good() )
            {
                const Eigen::Vector2d* p = is.read();
                if( !p ) { break; }
                tied.append( pinhole.undistorted( *p ) );
            }
            return 0;
        }
        if( operation == "distort" )
        {
            output_details< Eigen::Vector2d, Eigen::Vector2d >( options );
            comma::csv::input_stream< Eigen::Vector2d > is( std::cin, csv );
            comma::csv::output_stream< Eigen::Vector2d > os( std::cout, csv.binary(), false, csv.flush );
            comma::csv::tied< Eigen::Vector2d, Eigen::Vector2d > tied( is, os );
            while( is.ready() || std::cin.good() )
            {
                const Eigen::Vector2d* p = is.read();
                if( !p ) { break; }
                tied.append( pinhole.distort( *p ) );
            }
            return 0;
        }
        if( operation=="distortion-map" )
        {
            if( !pinhole.distortion_map() ) { std::cerr << "image-pinhole: no distortion specified in config" << std::endl; return 1; }
            pinhole.distortion_map()->write( std::cout );
            return 0;
        }
        std::cerr << "image-pinhole: error: unrecognized operation: "<< operation << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << "image-pinhole: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "image-pinhole: unknown exception" << std::endl; }
}
