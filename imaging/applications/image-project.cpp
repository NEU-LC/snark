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

#include <boost/filesystem/operations.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/serialize.h>
#include "../camera/config.h"
#include "../camera/traits.h"

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "take on stdin pixels, append their cartesian coordinates in camera frame based on pinhole camera model" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat pixels.csv | image-project <options> > points.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: print help; --help --verbose for more help" << std::endl;
    std::cerr << "    --camera-config,--camera,--config,-c=<parameters>: camera configuration" << std::endl;
    std::cerr << "        <parameters>: filename of json configuration file or ';'-separated path-value pairs" << std::endl;
    std::cerr << "                      e.g: --config=\"focal_length/x=123;focal_length/y=123.1;...\"" << std::endl;
    std::cerr << "    --input-fields: output input fields" << std::endl;
    std::cerr << "    --output-config,--sample-config: output sample config and exit" << std::endl;
    std::cerr << "    --output-fields: output appended fields and exit" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << std::endl;
    if( verbose ) { std::cerr << comma::csv::options::usage() << std::endl; }
    exit( 0 );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--output-config,--sample-config" ) ) { comma::write_json( snark::camera::config(), std::cout ); return 0; }
        if( options.exists( "--input-fields" ) ) { std::cerr << comma::join( comma::csv::names< Eigen::Vector2d >(), ',' ) << std::endl; return 0; }
        if( options.exists( "--output-fields" ) ) { std::cerr << comma::join( comma::csv::names< Eigen::Vector3d >(), ',' ) << std::endl; return 0; }
        std::string config_parameters = options.value< std::string >( "--camera-config,--camera,--config,-c" );
        snark::camera::config config;
        if( boost::filesystem::exists( config_parameters ) ) { comma::read_json( config, config_parameters ); }
        else
        {
            // todo
        }
        comma::csv::options csv( options );
        comma::csv::input_stream< Eigen::Vector2d > is( std::cin, csv );
        comma::csv::options output_csv;
        if( csv.binary() ) { output_csv.format( comma::csv::format::value< Eigen::Vector3d >() ); }
        comma::csv::output_stream< Eigen::Vector3d > os( std::cout, output_csv );
        comma::csv::tied< Eigen::Vector2d, Eigen::Vector3d > tied( is, os );
        while( is.ready() || std::cin.good() )
        {
            const Eigen::Vector2d* p = is.read();
            if( !p ) { break; }
            // todo
        }
    }
    catch( std::exception& ex ) { std::cerr << "image-project: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "image-project: unknown exception" << std::endl; }
}
