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

#include <stdlib.h>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <Eigen/Core>
#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/stream.h>
#include "../qt3d/qt3d_v2/types.h"

static void bash_completion( unsigned const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h";

    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << "\nOutput test data";
    std::cerr << "\n";
    std::cerr << "\nUsage: " << comma::verbose.app_name() << " <mode> [<options>]";
    std::cerr << "\nwhere <mode> is \"cube\" (only mode provided at the moment)";
    std::cerr << "\n    required parameters: count, width, thickness";
    std::cerr << "\n    e.g. " << comma::verbose.app_name() << " cube 100000 0.1 0.02";
    std::cerr << "\n";
    std::cerr << "\nOptions: ";
    std::cerr << "\n    --help,-h:          show this help, --help --verbose for more help";
    std::cerr << "\n";
    std::cerr << std::endl;
    exit( 0 );
}

float p2c( float p, float width, float thickness )
{
    return fabs( p / ( width + thickness ));
}

float random( float min, float max )
{
    return (float)::random() / RAND_MAX * ( max - min ) + min;
}

int random_sign()
{
    return ::random() % 2 * 2 - 1;
}

Eigen::Vector3f make_point( float width, float thickness )
{
    float min = width - thickness;
    float max = width + thickness;
    float x_min = 0;
    float y_min = 0;
    float z_min = 0;
    switch( ::random() % 3 )
    {
        case 0: x_min = min; break;
        case 1: y_min = min; break;
        case 2: z_min = min; break;
    }
    float x = random_sign() * random( x_min, max );
    float y = random_sign() * random( y_min, max );
    float z = random_sign() * random( z_min, max );
    return Eigen::Vector3f( x, y, z );
}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" )) { bash_completion( argc, argv ); }

        comma::csv::options output_csv;
        comma::csv::output_stream< snark::graphics::qt3d::vertex_t > ostream( std::cout, output_csv );

        std::vector< std::string > unnamed = options.unnamed( "--help,h", "-.*,--.*" );
        std::string mode = unnamed[0];

        if( mode == "cube" )
        {
            if( unnamed.size() != 4 ) { std::cerr << comma::verbose.app_name() << ": cube mode requires three arguments: num_points, width, thickness" << std::endl; return 1; }

            unsigned int num_points = boost::lexical_cast< unsigned int >( unnamed[1] );
            float width = boost::lexical_cast< float >( unnamed[2] );
            float thickness = boost::lexical_cast< float >( unnamed[3] );
            for( unsigned int i = 0; i < num_points; i++ )
            {
                Eigen::Vector3f p = make_point( width, thickness );
                snark::graphics::qt3d::vertex_t vertex( p, snark::graphics::qt3d::gl_color_t
                                                      ( p2c( p.x(), width, thickness )
                                                      , p2c( p.y(), width, thickness )
                                                      , p2c( p.z(), width, thickness ), 1.0 ));
                ostream.write( vertex );
            }
        }
        else
        {
            std::cerr << comma::verbose.app_name() << ": mode must be \"cube\"" << std::endl;
            return 1;
        }
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl;
        return 1;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl;
        return 1;
    }
    return 0;
}
