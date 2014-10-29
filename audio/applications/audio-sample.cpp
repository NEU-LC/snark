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

/// @author vsevolod vlaskine

#include <cmath>
#include <iostream>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/visiting/traits.h>
#include <snark/visiting/traits.h>

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "todo" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: audio-sample <options> > sample.pcm" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: --help --verbose for more help" << std::endl;
    std::cerr << "    --duration=[<seconds>]: if duration field absent, use this duration for all the samples" << std::endl;
    std::cerr << "    todo" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    if( verbose ) { std::cerr << std::endl << "csv options" << std::endl << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    echo -e 440,15000,2,0\\n880,10000,2,0\\n1760,12000,2,0 | audio-sample -r 64000 | csv-to-bin d | csv-cast d w --force > test.64000.w.raw" << std::endl;
    std::cerr << "    sox -r 64k -e signed -b 16 -c 1 test.64000.w.raw test.64000.w.wav" << std::endl;
    std::cerr << "    play test.64000.w.wav" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

struct input
{
    //boost::posix_time::ptime time;
    double frequency;
    double amplitude;
    double duration;
    comma::uint32 block;
    
    input() : block( 0 ) {}
};

namespace comma { namespace visiting {

template <> struct traits< input >
{
    template < typename K, typename V > static void visit( const K&, input& n, V& v )
    {
        //v.apply( "time", n.time );
        v.apply( "frequency", n.frequency );
        v.apply( "amplitude", n.amplitude );
        v.apply( "duration", n.duration );
        v.apply( "block", n.block );
    }

    template < typename K, typename V > static void visit( const K&, const input& n, V& v )
    {
        //v.apply( "time", n.time );
        v.apply( "frequency", n.frequency );
        v.apply( "amplitude", n.amplitude );
        v.apply( "duration", n.duration );
        v.apply( "block", n.block );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        bool verbose = options.exists( "--verbose,-v" );
        unsigned int rate = options.value< unsigned int >( "--rate,-r" );
        comma::csv::options csv( options );
        input default_input;
        default_input.duration = options.value( "--duration", 0.0 );
        comma::csv::input_stream< input > istream( std::cin, csv, default_input );
        boost::optional< input > last;
        std::vector< input > v;
        unsigned int count = 0;
        while( std::cin.good() )
        {
            const input* p = istream.read();
            if( !p || ( !v.empty() && v.back().block != p->block ) )
            {
                double step = 1.0 / rate;
                for( double t = 0; t < v[0].duration; t += step )
                {
                    double a = 0;
                    for( unsigned int i = 0; i < v.size(); ++i ) { a += v[i].amplitude * std::sin( M_PI * 2 * v[i].frequency * t ); }
                    if( csv.binary() ) { std::cout.write( reinterpret_cast< const char* >( &a ), sizeof( double ) ); }
                    else { std::cout << a << std::endl; }
                }
                v.clear();
                if( verbose && ++count % 100 == 0 ) { std::cerr << "audio-sample: processed " << count << " blocks" << std::endl; }
            }
            if( !p ) { break; }
            if( !v.empty() && !comma::math::equal( v.back().duration, p->duration ) ) { std::cerr << "audio-sample: expected consistent duration across a block, got " << v.back().duration << " and " << p->duration << " in block" << p->block << std::endl; return 1; }
            v.push_back( *p );
        }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << "audio-sample: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "audio-sample: unknown exception" << std::endl;
    }
    return 1;
}
