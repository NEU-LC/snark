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

/// @author vsevolod vlaskine

#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/parser.h>
#include <comma/csv/options.h>
#include "../cv_mat/serialization.h"
    
struct input_t
{
    boost::posix_time::ptime t;
    double x;
    double y;
    std::vector< double > channels;
    comma::uint32 block;
    
    input_t() : x( 0 ), y( 0 ), block( 0 ) {}
};

namespace comma { namespace visiting {

template <> struct traits< input_t >
{
    template < typename K, typename V > static void visit( const K&, input_t& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "x", r.x );
        v.apply( "y", r.y );
        v.apply( "channels", r.channels );
        v.apply( "block", r.block );
    }
    
    template < typename K, typename V > static void visit( const K&, const input_t& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "x", r.x );
        v.apply( "y", r.y );
        v.apply( "channels", r.channels );
        v.apply( "block", r.block );
    }
};

} } // namespace comma { namespace visiting {

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "read pixel coordinates and values, output images in the format readable by cv-cat" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: image-from-csv <options>" << std::endl
              << std::endl
              << "options" << std::endl
              << "    --help,-h: show help; --help --verbose: more help" << std::endl
              << "    --from,--begin,--origin=[<x>,<y>]: offset pixel coordinates by a given offset; default: 0,0" << std::endl
              << "    --output: output options, same as --input for image-from-csv or cv-cat (see --help --verbose)" << std::endl
              << "    --verbose,-v: more output" << std::endl
              << std::endl
              << "fields: t,x,y,r,g,b,block or t,x,y,grey,block" << std::endl
              << "    t: image timestamp, optional" << std::endl
              << "    x,y: pixel coordinates, if double, will get rounded to the nearest integer" << std::endl
              << "    r,g,b: red, green, blue values" << std::endl
              << "    grey: greyscale value" << std::endl
              << "    channels[0],channels[1],channels[2]: blue, green, red values; notice that the order is bgr" << std::endl
              << "                                         if only channels[0] given, it is the same as specifying grey field" << std::endl
              << "    block: image number, optional" << std::endl
              << std::endl;
    if( verbose )
    {
        std::cerr<< snark::cv_mat::serialization::options::usage() << std::endl << std::endl;
        std::cerr<< "input stream csv options" << std::endl << comma::csv::options::usage() << std::endl << std::endl;
    }
    std::cerr << "example" << std::endl;
    std::cerr << "     cat pixels.csv | image-from-csv --fields x,y,grey --output=\"rows=1200;cols=1000;type=ub\" | cv-cat --output no-header \"encode=png\" > test.bmp" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

static void set_pixel( cv::Mat& m, const input_t& v, const std::pair< double, double >& offset ) // quick and dirty; reimplement as templates
{
    int x = std::floor( v.x + 0.5 - offset.first );
    int y = std::floor( v.y + 0.5 - offset.second );
    if( x < 0 || x >= m.cols ) { return; }
    if( y < 0 || y >= m.rows ) { return; }
    switch( m.type() )
    {
        case CV_8UC1:
            m.at< unsigned char >( y, x ) = v.channels[0];
            break;
        case CV_32FC1:
            m.at< float >( y, x ) = v.channels[0];
            break;
        case CV_8UC3:
        {
            cv::Vec3b& p = m.at< cv::Vec3b >( y, x );
            p[0] = v.channels[0];
            p[1] = v.channels[1];
            p[2] = v.channels[2];
            break;
        }
        case CV_32FC3:
        {
            cv::Vec3f& p = m.at< cv::Vec3f >( y, x );
            p[0] = v.channels[0];
            p[1] = v.channels[1];
            p[2] = v.channels[2];
            break;
        }
        default:
            COMMA_THROW( comma::exception, "unsupported cv mat type " << m.type() );
    }   
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv( options );
        if( csv.fields.empty() ) { std::cerr << "image-from-csv: please specify --fields" << std::endl; return 1; }
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        input_t sample;
        bool is_greyscale = true;
        bool has_alpha = false;
        for( unsigned int i = 0; i < v.size(); ++i )
        {
            if( v[i] == "grey" ) { v[i] = "channels[0]"; }
            else if( v[i] == "b" ) { v[i] = "channels[0]"; is_greyscale = false; }
            else if( v[i] == "g" ) { v[i] = "channels[1]"; is_greyscale = false; }
            else if( v[i] == "r" ) { v[i] = "channels[2]"; is_greyscale = false; }
            else if( v[i] == "a" ) { v[i] = "channels[3]"; is_greyscale = false; has_alpha = true; }
        }
        csv.fields = comma::join( v, ',' );
        if( has_alpha ) { std::cerr << "image-from-csv: alpha support: todo" << std::endl; return 1; }
        std::string offset_string = options.value< std::string >( "--from,--begin,--origin", "0,0" );
        const std::vector< std::string >& w = comma::split( offset_string, ',' );
        if( w.size() != 2 ) { std::cerr << "image-from-csv: --from: expected <x>,<y>; got: \"" << offset_string << "\"" << std::endl; return 1; }
        std::pair< double, double > offset( boost::lexical_cast< double >( w[0] ), boost::lexical_cast< double >( w[1] ) ); // todo: quick and dirty; use better types
        if( is_greyscale && has_alpha ) { std::cerr << "image-from-csv: warning: found alpha channel for a greyscale image; not implemented; ignored" << std::endl; }
        sample.channels.resize( is_greyscale ? 1 : has_alpha ? 4 : 3 );
        snark::cv_mat::serialization::options output_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( options.value<std::string>("--output" ) );
        // todo: check whether output type matches fields
        snark::cv_mat::serialization output( output_options );
        comma::csv::input_stream< input_t > is( std::cin, csv, sample );
        boost::optional< input_t > last;
        int type = output_options.get_header().type;
        while( is.ready() || std::cin.good() )
        {
            std::pair< boost::posix_time::ptime, cv::Mat > pair;
            pair.second = cv::Mat::zeros( output_options.rows, output_options.cols, type );
            if( last ) { set_pixel( pair.second, *last, offset ); }
            while( is.ready() || std::cin.good() )
            {
                const input_t* p = is.read();
                bool block_done = last && ( !p || p->block != last->block );
                if( p ) { last = *p; }
                if( block_done )
                {
                    pair.first = last->t;
                    output.write( std::cout, pair );
                    std::cout.flush();
                    break;
                }
                if( !p ) { break; }
                set_pixel( pair.second, *p, offset );
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "image-from-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "image-from-csv: unknown exception" << std::endl; }
    return 1;
}
