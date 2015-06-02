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

#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <comma/application/signal_flag.h>
#include "control.h"
#include "pid.h"

static const char* name() { return "control-command"; }

template< typename T > std::string field_names( bool full_xpath = false ) { return comma::join( comma::csv::names< T >( full_xpath ), ',' ); }
template< typename T > std::string format( std::string fields, bool full_xpath = false ) { return comma::csv::format::value< T >( fields, full_xpath ); }

typedef snark::control::control_data_t control_data_t;
typedef snark::control::command_t command_t;

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "take control data on stdin and output command to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: " << name() << " [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --cross-track-pid=<p>,<i>,<d>[,<error threshold>]: cross track pid parameters" << std::endl;
    std::cerr << "    --heading-pid=<p>,<i>,<d>[,<error threshold>]: heading pid parameters" << std::endl;
    std::cerr << "    --control-type <type>: control type (available types: " << "todo" << ")" << std::endl;
    std::cerr << "    --format: show default binary format of input stream and exit" << std::endl;
    std::cerr << "    --output-format: show binary format of output stream and exit" << std::endl;
    std::cerr << "    --output-fields: show output fields and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "csv options:" << std::endl;
    std::cerr << comma::csv::options::usage( field_names< control_data_t >( true) ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    cat targets.csv | control-error \"feedback\" |" << name() << " --cross-track-pid=1,0,0 --heading-pid=1,0,0" << std::endl;
    exit( 1 );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options input_csv( options, field_names< control_data_t >( true ) );
        input_csv.full_xpath = true;
        comma::csv::input_stream< control_data_t > input_stream( std::cin, input_csv );
        comma::csv::options output_csv( options );
        output_csv.fields = field_names< command_t >();
        if( input_csv.binary() ) { output_csv.format( format< command_t >( output_csv.fields ) ); }
        comma::csv::output_stream< command_t > output_stream( std::cout, output_csv );
        if( options.exists( "--format" ) ) { std::cout << format< control_data_t >( input_csv.fields, true ) << std::endl; return 0; }
        if( options.exists( "--output-format" ) ) { std::cout << format< command_t >( output_csv.fields ) << std::endl; return 0; }
        if( options.exists( "--output-fields" ) ) { std::cout << output_csv.fields << std::endl; return 0; }
        snark::control::pid<> cross_track_pid( options.value< std::string >( "--cross-track-pid" ) );
        snark::control::pid<> heading_pid( options.value< std::string >( "--heading-pid" ) );
        comma::signal_flag is_shutdown;
        while( !is_shutdown && ( input_stream.ready() || ( std::cin.good() && !std::cin.eof() ) ) )
        {
            const control_data_t* control = input_stream.read();
            if( !control ) { break; }
            command_t command;
            command.turn_rate = cross_track_pid.update( control->error.heading, control->feedback.t );
            double local_heading = heading_pid.update( control->error.cross_track, control->feedback.t );
            double yaw = control->feedback.data.orientation.yaw;
            double heading_offset = control->target.parameters.heading_offset;
            command.local_heading = snark::control::angle_wrap( yaw + heading_offset - local_heading );
            output_stream.write( command );
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << name() << ": unknown exception" << std::endl; }
    return 1;
}