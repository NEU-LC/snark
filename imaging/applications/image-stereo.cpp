// This file is part of snark, a generic and flexible library for robotics research
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

/// @author vsevolod vlaskine

#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/name_value/serialize.h>
#include "../../visiting/traits.h"
#include "../camera/stereo.h"
#include "../camera/traits.h"

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "take on stdin pixels, perform an operation based a pair of pinhole cameras" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat pixels.csv | image-stereo <operation> <options> > points.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    to-cartesian: take on stdin pixels on stereo pair, undistort image, append pixel's cartesian coordinates in a given frame" << std::endl;
    std::cerr << "        fields: default: first/x,first/y,second/x,second/y" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --force,--permissive: discard invalid input instead of exiting with error" << std::endl;
    std::cerr << "    --input-fields: output input fields to stdout and exit" << std::endl;
    std::cerr << "    --output-fields: output appended fields for given operation and exit" << std::endl;
    std::cerr << "    --output-format: output appended fields for given operation and exit" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "configuration options" << std::endl;
    std::cerr << "    --baseline=[<meters>]: convenience option, todo" << std::endl;
    std::cerr << "    --camera-config=<path>: camera config for both cameras; <path>: <filename> or <filename>:<path>" << std::endl;
    std::cerr << "    --config,-c=<path>: camera pair config; <path>: <filename> or <filename>:<path>" << std::endl;
    std::cerr << "    --config-fields: output config fields to stdout and exit" << std::endl;
    std::cerr << "    --first-camera-config,--first-config=<path>: first camera config; <path>: <filename> or <filename>:<path>" << std::endl;
    std::cerr << "    --first-pose=<x,y,z,roll,pitch,yaw>: pose of the first camera; default: whatever is in camera config" << std::endl;
    std::cerr << "    --second-camera-config,--second-config=<path>: second camera config; <path>: <filename> or <filename>:<path>" << std::endl;
    std::cerr << "    --second-pose=<x,y,z,roll,pitch,yaw>: pose of the second camera; default: whatever is in camera config" << std::endl;
    std::cerr << std::endl;
    std::cerr << "csv options" << std::endl;
    std::cerr << comma::csv::options::usage( verbose ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    todo" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

template < typename S, typename T > static void output_details( const comma::command_line_options& options )
{
    if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< S >(), ',' ) << std::endl; exit( 0 ); }
    if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< T >(), ',' ) << std::endl; exit( 0 ); }
    if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format::value< T >() << std::endl; exit( 0 ); }
}

template < typename T >
static T read_config( const std::string& config_parameters )
{
    T config;
    const std::vector< std::string >& v = comma::split( config_parameters, ":#@" );
    if( v.size() == 1 ) { comma::read( config, config_parameters, true ); }
    else { comma::read( config, config_parameters.substr( 0, config_parameters.size() - v.back().size() - 1 ), v.back(), true ); }
    return config;
}

static snark::camera::pinhole::config_t make_sample_pinhole_config()
{
    snark::camera::pinhole::config_t config;
    config.focal_length = 0.1;
    config.image_size = Eigen::Vector2i( 1000, 2000 );
    config.sensor_size = Eigen::Vector2d( 0.1, 0.2 );
    config.distortion = snark::camera::pinhole::config_t::distortion_t( snark::camera::pinhole::config_t::distortion_t::radial_t( 0.001, -0.0002, 0.003 ), snark::camera::pinhole::config_t::distortion_t::tangential_t( 0.0004, -0.0005 ) );
    return config;
}

static snark::camera::stereo::pair::config_t make_sample_config()
{
    snark::camera::stereo::pair::config_t config;
    config.first.pinhole = config.second.pinhole = make_sample_pinhole_config();
    config.first.pose = config.second.pose = snark::pose( Eigen::Vector3d( 1, 2, 3 ), snark::roll_pitch_yaw( 0.1, 0.2, 0.3 ) );    
    return config;
}

static snark::camera::stereo::pair make_pair( const comma::command_line_options& options )
{
    options.assert_mutually_exclusive( "--config,-c", "--camera-config,--first-camera-config,--first-config,--second-camera-config,--second-config" );
    options.assert_mutually_exclusive( "--camera-config", "--first-camera-config,--first-config,--second-camera-config,--second-config" );
    if( options.exists( "--baseline" ) ) { std::cerr << "image-stereo: --baseline: not implemented" << std::endl; exit( 1 ); }
    snark::camera::stereo::pair::config_t config;
    if( options.exists( "--config,-c" ) )
    { 
        config = read_config< snark::camera::stereo::pair::config_t >( options.value< std::string >( "--config,-c" ) );
    }
    else if( options.exists( "--camera-config" ) )
    {
        config.first.pinhole = config.second.pinhole = read_config< snark::camera::pinhole::config_t >( options.value< std::string >( "--camera-config" ) );
    }
    else if( options.exists( "--first-camera-config,--first-config" ) )
    {
        config.first.pinhole = read_config< snark::camera::pinhole::config_t >( options.value< std::string >( "--first-camera-config,--first-config" ) );
        config.second.pinhole = read_config< snark::camera::pinhole::config_t >( options.value< std::string >( "--second-camera-config,--second-config" ) );
    }
    config.first.pinhole.validate();
    config.second.pinhole.validate();
    auto first_pose = options.optional< std::string >( "--first-pose" );
    if( first_pose ) { config.first.pose = comma::csv::ascii< snark::pose >().get( first_pose ); }
    auto second_pose = options.optional< std::string >( "--second-pose" );
    if( second_pose ) { config.second.pose = comma::csv::ascii< snark::pose >().get( second_pose ); }
    return snark::camera::stereo::pair( config );
}

struct point_t: public Eigen::Vector2d
{
    snark::pose pose;
    point_t(): Eigen::Vector2d( Eigen::Vector2d::Zero() ) {}
};

typedef std::pair< point_t, point_t > input_t;

namespace comma { namespace visiting {

template <> struct traits< point_t >
{
    template < typename Key, class Visitor > static void visit( const Key& k, point_t& t, Visitor& v )
    {
        comma::visiting::traits< Eigen::Vector2d >::visit( k, t, v );
        v.apply( "pose", t.pose );
    }

    template < typename Key, class Visitor > static void visit( const Key& k, const point_t& t, Visitor& v )
    {
        comma::visiting::traits< Eigen::Vector2d >::visit( k, t, v );
        v.apply( "pose", t.pose );
    }
};
    
} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--config-fields" ) ) { std::cout << comma::join( comma::csv::names( true, make_sample_config() ), '\n' ) << std::endl; return 0; }
        const std::vector< std::string >& unnamed = options.unnamed( "--force,--permissive,--input-fields,--output-fields,--output-format,--flush", "-.*" );
        if( unnamed.empty() ) { std::cerr << "image-stereo: please specify operation" << std::endl; return 1; }
        std::string operation = unnamed[0];
        comma::csv::options csv( options );
        csv.full_xpath = true;
        bool force = options.exists( "--force,--permissive" );
        if( operation == "to-cartesian" )
        {
            if( csv.fields.empty() ) { csv.fields = "first/x,first/y,second/x,second/y"; }
            output_details< input_t, std::pair< Eigen::Vector3d, Eigen::Vector3d > >( options );
            auto pair = make_pair( options );
            input_t sample;
            sample.first.pose = pair.first().pose;
            sample.second.pose = pair.second().pose;
            comma::csv::input_stream< input_t > is( std::cin, csv, sample );
            comma::csv::output_stream< std::pair< Eigen::Vector3d, Eigen::Vector3d > > os( std::cout, csv.binary(), false, csv.flush );
            comma::csv::tied< input_t, std::pair< Eigen::Vector3d, Eigen::Vector3d > > tied( is, os );
            while( is.ready() || std::cin.good() )
            {
                const input_t* p = is.read();
                if( !p ) { break; }
                if( comma::math::equal( ( p->first.pose.translation - p->second.pose.translation ).norm(), 0.01 ) )
                {
                    if( force ) { std::cerr << "image-stereo: expected two camera positions with sufficient paralax; got first: " << p->first.pose.translation.transpose() << " second: " << p->second.pose.translation.transpose() << "; discarded" << std::endl; }
                    else { std::cerr << "image-stereo: expected two camera positions with sufficient paralax; got first: " << p->first.pose.translation.transpose() << " second: " << p->second.pose.translation.transpose() << "; use --force to override" << std::endl; return 1; }
                }
                tied.append( pair.to_cartesian( p->first, p->second, p->first.pose, p->second.pose ) );
            }
            return 0;
        }
        std::cerr << "image-stereo: expected operation; got: '"<< operation << "'" << std::endl;
    }
    catch( std::exception& ex ) { std::cerr << "image-stereo: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "image-stereo: unknown exception" << std::endl; }
    return 1;
}
