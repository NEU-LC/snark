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
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#include <cmath>
#include <iostream>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "convert angles in degrees to radians" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    math-deg2rad <angle> <angle> ..." << std::endl;
    std::cerr << "    cat values.csv | math-deg2rad --fields=,,,alpha,beta" << std::endl;
    std::cerr << "    cat values.bin | math-deg2rad --fields=,b,a --binary=t,2d,ui" << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

struct Line
{
    enum { size = 1024 };
    boost::array< double, size > values;
};

static const long double ratio = M_PI / ( long double )( 180 );

namespace comma { namespace visiting {

template <> struct traits< Line >
{
    template < typename K, typename V > static void visit( const K&, Line& p, V& v )
    {
        v.apply( "values", p.values );
    }

    template < typename K, typename V > static void visit( const K&, const Line& p, V& v )
    {
        v.apply( "values", p.values );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    comma::command_line_options options( ac, av );
    if( options.exists( "--help,-h" ) ) { usage(); }
    std::vector< std::string > unnamed = options.unnamed( "", "--delimiter,-d,--binary,-b,--fields,-f" );
    if( unnamed.empty() )
    {
        comma::csv::options csv( options );
        if( csv.fields == "" ) { std::cerr << "math-deg2rad: please specify --fields (todo: handle default)" << std::endl; return 1; }
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        if( v.size() > Line::size ) { std::cerr << "math-deg2rad: expected not more than 1024 --fields (todo: handle arbitrary number of fields)" << std::endl; return 1; }
        unsigned int count = 0; // quick and dirty
        for( std::size_t i = 0; i < v.size(); ++i )
        {
            if( !v[i].empty() ) { v[i] = "values[" + boost::lexical_cast< std::string >( count++ ) + "]"; }
        }
        csv.fields = comma::join( v, ',' );
        comma::csv::input_stream< Line > istream( std::cin, csv );
        comma::csv::output_stream< Line > ostream( std::cout, csv );
        comma::signal_flag is_shutdown;
        while( !is_shutdown && std::cin.good() && !std::cin.eof() )
        {
            const Line* input = istream.read();
            if( !input ) { break; }
            Line output = *input; // quick and dirty
            for( unsigned int i = 0; i < count; output.values[i] = ratio * output.values[i], ++i );
            if( csv.binary() ) { ostream.binary().write( output, istream.binary().last() ); }
            else { ostream.ascii().write( output, istream.ascii().last() ); }
        }
    }
    else
    {
        std::cout.precision( 12 );
        std::cout << ( ratio * boost::lexical_cast< double >( unnamed[0] ) );
        for( unsigned int k = 1; k < unnamed.size(); ++k )
        {
            std::cout << " " << ( ratio * boost::lexical_cast< double >( unnamed[k] ) );
        }
        std::cout << std::endl;
    }
    return 0;
}
