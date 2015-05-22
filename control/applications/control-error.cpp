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

#include <iostream>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/csv/names.h>
#include <comma/csv/traits.h>
#include <comma/visiting/traits.h>
#include <comma/io/select.h>
#include <comma/io/stream.h>
#include <comma/name_value/name_value.h>
#include "control_error.h"

static const char* name() { return "control-error"; }

template< typename T > std::string field_names() { return comma::join( comma::csv::names< T >( false ), ',' ); }
static const std::string default_fields = "x,y";
static const double default_proximity = 0.1;

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "take waypoints on stdin and copy to stdout with the appended cross-track and heading errors with respect to the feedback stream" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: " << name() << " <feedback> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "feedback: \"<address>[;<options>]\"" << std::endl;
    std::cerr << "    address: address where the feedback stream is published, e.g. tcp:localhost:12345" << std::endl;
    std::cerr << "    options: semicolon-separated csv options of the feedback stream, e.g. fields=t,x,y,,,,yaw;binary=t,6d" << std::endl;
    std::cerr << "    default fields: " << field_names< feedback_t >() << std::endl;
    std::cerr << std::endl;
    std::cerr << "input stream options: " << std::endl;
    std::cerr << "    --fields,-f <names>: comma-separated field names (possible fields: " << field_names< input_t >() << "; default: " << default_fields << ")" << std::endl;
    std::cerr << "    --binary,-b <format>: use binary format" << std::endl;
    if( verbose ) { std::cerr << comma::csv::format::usage() << std::endl; }
    std::cerr << std::endl;
    std::cerr << "control error stream options: " << std::endl;
    std::cerr << "    --error-fields <names>: comma-separated field names (default: " << field_names< control_error_t >() << ")" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options: " << std::endl;
    std::cerr << "    --proximity: a waypoint is reached as soon as position is within proximity (default: " << default_proximity << ")" << std::endl;
    std::cerr << "    --format: output binary format of input stream to stdout and exit" << std::endl;
    std::cerr << "    --output-format: output binary format of output stream to stdout and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "notes: " << std::endl;
    std::cerr << "    1) the control errors are determined with respect to the current wayline, which in turn is determined by the last two waypoints" << std::endl;
    std::cerr << "    2) the first wayline is from the first feedback position to the first waypoint" << std::endl;
    std::cerr << "    3) the transition to the next wayline occurs when the waypoint at the end of the current wayline is reached" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples: " << std::endl;
    std::cerr << "    cat waypoints.bin | " << name() << " \"tcp:localhost:12345;fields=t,x,y,,,,yaw;binary=t,6d\" --fields=x,,,y,speed --binary=3d --error-fields=heading,cross_track > control_errors.bin" << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options input_csv( options, default_fields );
        comma::csv::input_stream< input_t > input_stream( std::cin, input_csv );
        comma::csv::options error_csv;
        if( options.exists( "--error-fields" ) ) { error_csv.fields = options.value< std::string >( "--error-fields" ); }
        if( input_csv.binary() ) { error_csv.format( comma::csv::format::value< control_error_t >() ); }
        comma::csv::output_stream< control_error_t > error_stream( std::cout, error_csv );
        comma::csv::tied< input_t, control_error_t > tied( input_stream, error_stream );
        if( options.exists( "--format" ) ) { std::cerr << comma::csv::format::value< input_t >( input_csv.fields, false ) << std::endl; return 0; }
        if( options.exists( "--error-format" ) ) { std::cerr << comma::csv::format::value< control_error_t >( error_csv.fields, false ) << std::endl; return 0; }
        double proximity = options.value< double >( "--proximity", default_proximity );
        bool verbose = options.exists( "--verbose,-v" );
        std::vector< std::string > unnamed = options.unnamed( "--help,-h,--verbose,-v,--format,--output-format", "--fields,-f,--binary,-b,--output-fields" );
        if( unnamed.empty() ) { std::cerr << name() << ": feedback stream is not given" << std::endl; return 1; }
        comma::csv::options feedback_csv = comma::name_value::parser( "filename", ';', '=', false ).get< comma::csv::options >( unnamed[0] );
        if( feedback_csv.fields.empty() ) { feedback_csv.fields = field_names< feedback_t >(); }
        comma::io::istream feedback_in( feedback_csv.filename, feedback_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii, comma::io::mode::non_blocking );
        comma::csv::input_stream< feedback_t > feedback_stream( *feedback_in, feedback_csv );
        comma::io::select select;
        select.read().add( comma::io::stdin_fd );
        select.read().add( feedback_in );
        boost::optional< feedback_t > feedback;
        boost::optional< input_t > input;
        boost::optional< vector_t > to;
        boost::optional< vector_t > from;
        wayline_t wayline;
        comma::signal_flag is_shutdown;
        while( !is_shutdown && std::cin.good() && !std::cin.eof() )
        {
            select.check();
            if( input_stream.ready() || select.read().ready( comma::io::stdin_fd ) )
            {
                const input_t* i = input_stream.read();
                if( !i ) { break; }
                input = *i;
                to = input->location;
                if( verbose ) { std::cerr << name() << ": received target waypoint " << serialise( *to ) << std::endl; }
                if( from && distance( *from, *to ) < proximity )
                {
                    if( verbose ) { std::cerr << name() << ": skipping waypoint " << serialise( *to ) << " since it is within " << proximity << " of previous waypoint " << serialise( *from ) << std::endl; }
                    continue;
                }
                if( from ) { wayline = wayline_t( *from, *to, verbose ); }
                while( !is_shutdown && std::cout.good() )
                {
                    select.check();
                    if( feedback_stream.ready() || select.read().ready( feedback_in ) )
                    {
                        const feedback_t* f = feedback_stream.read();
                        if( !f ) { std::cerr << name() << ": feedback stream error occurred prior to reaching waypoint " << serialise( *to ) << std::endl; return 1; }
                        feedback = *f;
                        position_t current = feedback->data;
                        if( distance( current.location, *to ) < proximity )
                        {
                            if( verbose ) { std::cerr << name() << ": waypoint " << serialise( *to ) << " is reached (proximity)" << std::endl; }
                            from = *to;
                            break;
                        }
                        if( !from )
                        {
                            from = current.location; // the first feedback point is the start point of the first wayline
                            wayline = wayline_t( *from, *to, verbose );
                        }
                        if( wayline.is_past_endpoint( current.location ) )
                        {
                            if( verbose ) { std::cerr << name() << ": waypoint " << serialise( *to ) << " is reached (past endpoint)" << std::endl; }
                            from = *to;
                            break;
                        }
                        control_error_t error;
                        error.cross_track = wayline.cross_track_error( current.location );
                        error.heading = wayline.heading_error( current.orientation.yaw );
                        tied.append( error );
                    }
                }
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << name() << ": unknown exception" << std::endl; }
    return 1;
}
