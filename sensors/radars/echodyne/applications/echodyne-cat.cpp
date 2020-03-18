// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2020 Mission Systems Pty Ltd
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright owner nor the names of the contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
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

#include <chrono>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include "../../../../visiting/traits.h"
#include "../echodyne.h"
#include "../radar.h"
#include "../reader.h"
#include "../traits.h"

static void bash_completion( unsigned int const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " --output-fields --output-format"
        " --sample-data"
        " status rvmap detection track measurement"
        ;
    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << "\nStream one of the echodyne radar channels";
    std::cerr << "\n";
    std::cerr << "\nUsage: " << comma::verbose.app_name() << " <channel> [<options>]";
    std::cerr << "\n";
    std::cerr << "\n    where <channel> is one of:";
    std::cerr << "\n        status, rvmap, detection, track, measurement";
    std::cerr << "\n";
    std::cerr << "\nOptions:";
    std::cerr << "\n    --help,-h:             show this help";
    std::cerr << "\n    --verbose,-v:          more output to stderr";
    std::cerr << "\n    --output-fields:       print output fields and exit";
    std::cerr << "\n    --output-format:       print output format and exit";
    std::cerr << "\n    --sample-data=[<dir>]; read saved data from <dir>";
    std::cerr << "\n";
    std::cerr << "\nExamples:";
    std::cerr << "\n    " << comma::verbose.app_name() << " status | csv-from-bin $( " << comma::verbose.app_name() << " status --output-format )";
    std::cerr << "\n    " << comma::verbose.app_name() << " track --sample-data=/path/to/data";
    std::cerr << "\n    " << comma::verbose.app_name() << " rvmap --output-format | csv-format collapse";
    std::cerr << "\n";
    std::cerr << std::endl;
    exit( 0 );
}

template< typename T >
struct app
{
    static std::string output_fields() { return comma::join( comma::csv::names< T >( true ), ',' ); }
    static std::string output_format() { return comma::csv::format::value< T >(); }

    static int run( mesa_data_t channel, const comma::command_line_options& options )
    {
        if( options.exists( "--output-fields" )) { std::cout << output_fields() << std::endl; return 0; }
        if( options.exists( "--output-format" )) { std::cout << output_format() << std::endl; return 0; }

        comma::csv::options output_csv;
        output_csv.format( comma::csv::format::value< T >() );
        comma::csv::binary_output_stream< T > os( std::cout, output_csv );
        comma::signal_flag is_shutdown;

        if( options.exists( "--sample-data" ))
        {
            snark::echodyne::reader reader( options.value< std::string >( "--sample-data" ));
            reader.output< T >( channel, os );
        }
        else
        {
            snark::echodyne::radar radar;

            radar.connect();
            radar.set_time();
            radar.enable_buffer( channel );

            while( !is_shutdown && std::cout.good() )
            {
                radar.output< T >( channel, os );
            }
        }
        if( is_shutdown ) { std::cerr << comma::verbose.app_name() << ": interrupted by signal" << std::endl; }

        return 0;
    }
};

int main( int argc, char** argv )
{
    using namespace std::chrono_literals;
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( argc, argv );

        std::vector< std::string > unnamed = options.unnamed( "--output-fields,--output-format,--verbose,-v", "-.*" );

        if( unnamed.size() == 1 )
        {
            mesa_data_t channel = snark::echodyne::mesa_data_from_string( unnamed[0] );

            switch( channel )
            {
                case STATUS_DATA:     return app< snark::echodyne::status_data_t >::run( channel, options );
                case RVMAP_DATA:      return app< snark::echodyne::rvmap_data_t >::run( channel, options );
                case DETECTION_DATA:  return app< snark::echodyne::detection_data_t >::run( channel, options );
                case TRACK_DATA:      return app< snark::echodyne::track_data_t >::run( channel, options );
                case MEAS_DATA:       return app< snark::echodyne::meas_data_t >::run( channel, options );
                case N_MESAK_DATA_TYPES: break;
            }
        }
        std::cerr << comma::verbose.app_name() << ": require a channel name" << std::endl;
        return 1;
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl;
    }
    return 1;
}
