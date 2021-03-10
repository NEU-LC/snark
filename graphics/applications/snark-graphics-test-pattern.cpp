// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
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
#include "../../visiting/eigen.h"

static void bash_completion( unsigned const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --output-fields --output-format --binary -b --seed";

    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << "\nOutput test data";
    std::cerr << "\n";
    std::cerr << "\nUsage: " << comma::verbose.app_name() << " <mode> [<options>]";
    std::cerr << "\n";
    std::cerr << "\nwhere <mode> is one of:";
    std::cerr << "\n    cube <count> <width> <thickness>";
    std::cerr << "\n";
    std::cerr << "\nOptions: ";
    std::cerr << "\n    --help,-h:       show this help, --help --verbose for more help";
    std::cerr << "\n    --output-fields: show output fields and exit";
    std::cerr << "\n    --output-format: show binary output format and exit";
    std::cerr << "\n    --binary,-b:     output in binary (default is ascii)";
    std::cerr << "\n    --seed=[<n>]:    seed for random generator (see srand); default is time";
    std::cerr << "\n";
    std::cerr << "\nExamples: ";
    std::cerr << "\n    " << comma::verbose.app_name() << " cube 100000 0.1 0.02";
    std::cerr << "\n";
    std::cerr << std::endl;
    exit( 0 );
}

unsigned char point_to_color( float p, float width, float thickness )
{
    return (unsigned char)( fabs( p / ( width + thickness )) * 256 );
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

struct color_t
{
    // Use the view-points standard for colour
    boost::array< unsigned char, 4 > rgba;

    color_t()
    {
        rgba[0] = 0; rgba[1] = 0; rgba[2] = 0; rgba[3] = 255;
    }

    color_t( unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha )
    {
        rgba[0] = red; rgba[1] = green; rgba[2] = blue; rgba[3] = alpha;
    }

    color_t( const color_t& color ) : rgba( color.rgba ) {}

    unsigned char red() const   { return rgba[0]; }
    unsigned char green() const { return rgba[1]; }
    unsigned char blue() const  { return rgba[2]; }
    unsigned char alpha() const { return rgba[3]; }
};

// Define our own vertex type using our colour type.
// A more flexible alternative is to use snark::graphics::qt3d::vertex_t
// but this type works better with view-points.
struct vertex_t
{
    Eigen::Vector3f position;
    color_t color;

    vertex_t() {}
    vertex_t( const Eigen::Vector3f& position, const color_t& color )
        : position( position ), color( color ) {}
};

namespace comma { namespace visiting {

template <> struct traits< color_t >
{
    template < typename Key, class Visitor >
    static void visit( Key, color_t& p, Visitor& v )
    {
        unsigned char red   = 0;
        unsigned char green = 0;
        unsigned char blue  = 0;
        unsigned char alpha = 255;
        v.apply( "r", red );
        v.apply( "g", green );
        v.apply( "b", blue );
        v.apply( "a", alpha );
        p = color_t( red, green, blue, alpha );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const color_t& p, Visitor& v )
    {
        v.apply( "r", p.red() );
        v.apply( "g", p.green() );
        v.apply( "b", p.blue() );
        v.apply( "a", p.alpha() );
    }
};

template <> struct traits< vertex_t >
{
    template < typename Key, class Visitor >
    static void visit( Key, vertex_t& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "color", p.color );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const vertex_t& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "color", p.color );
    }
};

} } // namespace comma { namespace visiting {

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" )) { bash_completion( argc, argv ); }

        if( options.exists( "--output-fields" ))
        {
            std::cout << comma::join( comma::csv::names< vertex_t >( false ), ',' ) << std::endl;
            exit( 0 );
        }
        if( options.exists( "--output-format" ))
        {
            std::cout << comma::csv::format::value< vertex_t >() << std::endl;
            exit( 0 );
        }

        unsigned seed=options.value<unsigned>("--seed",time(0));
        srand(seed);
        comma::csv::options output_csv;
        output_csv.full_xpath = false;
        if( options.exists( "--binary,-b" )) { output_csv.format( comma::csv::format::value< vertex_t >() ); }
        comma::csv::output_stream< vertex_t > ostream( std::cout, output_csv );

        std::vector< std::string > unnamed = options.unnamed( "--help,h", "-.*,--.*" );
        if( unnamed.empty() )
        {
            std::cerr << "Usage: " << comma::verbose.app_name() << " <mode> [<options>]" << std::endl;
            exit( 1 );
        }
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
                vertex_t vertex( p, color_t( point_to_color( p.x(), width, thickness )
                                           , point_to_color( p.y(), width, thickness )
                                           , point_to_color( p.z(), width, thickness )
                                           , 255 ));
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
