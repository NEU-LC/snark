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

#include <iostream>
#include <comma/application/command_line_options.h>
#include <comma/csv/impl/fieldwise.h>
#include <comma/csv/stream.h>
#include "../../../math/spherical_geometry/coordinates.h"
#include "../../../timing/timestamped.h"
#include "../bd9xx/stream.h"
#include "../bd9xx/gsof.h"

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "acquire data from trimble over bd9xx gsof protocol" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: netcat localhost 12345 | trimble-to-csv [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: output help; --help --verbose: more help" << std::endl;
    std::cerr << "    --fields=<fields>: output fields" << std::endl;
    std::cerr << "    --verbose,-v: more output to stderr" << std::endl;
    std::cerr << std::endl;
    std::cerr << "fields" << std::endl;
    std::cerr << "    default: position" << std::endl;
    std::cerr << std::endl;
    if( verbose ) { std::cerr << std::endl << "binary options" << std::endl << comma::csv::options::usage() << std::endl; }
    exit( 0 );
}

struct position
{
    // todo
};

struct output
{
    // todo
};

static void handle( const snark::trimble::bd9xx::gsof::transmission::const_iterator& it )
{
    switch( it->type() )
    {
        case snark::trimble::bd9xx::packets::gsof::records::current_time_utc::type:
            // todo
            break;
            
        // todo
            
        default:
            // todo
            break;
    }
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        snark::trimble::bd9xx::input_stream is( std::cin );
        while( std::cin.good() )
        {
            snark::trimble::bd9xx::gsof::transmission transmission;
            while( std::cin.good() && !transmission.complete() )
            {
                const snark::trimble::bd9xx::packet* p = is.read();
                if( !p ) { break; }
                transmission.append( p->body(), p->header().length() );
            }
            if( !transmission.complete() ) { continue; } // may be end of stream or out of sync
            for( snark::trimble::bd9xx::gsof::transmission::const_iterator it = transmission.begin(); it != transmission.end(); ++it )
            {
                handle( it );
            }
            
            // todo: output
            
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "trimble-to-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "trimble-to-csv: unknown exception" << std::endl; }
    return 1;
}
