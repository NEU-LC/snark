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
#include "control.h"

static const char* name() { return "control-error"; }

template< typename T > std::string field_names( bool full_xpath = false ) { return comma::join( comma::csv::names< T >( full_xpath ), ',' ); }
template< typename T > std::string format( std::string fields, bool full_xpath = false ) { return comma::csv::format::value< T >( fields, full_xpath ); }
static const double default_proximity = 0.1;
static const std::string default_mode = "fixed";

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "take target waypoints on stdin and output to stdout with the appended wayline, feedback, and control errors" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: " << name() << " <feedback> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "feedback: \"<address>[;<options>]\"" << std::endl;
    std::cerr << "    address: address where the feedback stream is published, e.g. tcp:localhost:12345" << std::endl;
    std::cerr << "    options: semicolon-separated csv options of the feedback stream, e.g. fields=t,x,y,,,,yaw;binary=t,6d" << std::endl;
    std::cerr << "    default fields: " << field_names< snark::control::feedback_t >() << std::endl;
    std::cerr << std::endl;
    std::cerr << "input stream options: " << std::endl;
    std::cerr << "    --fields,-f <names>: comma-separated field names (default: " << field_names< snark::control::target_t >() << ")" << std::endl;
    std::cerr << "    --binary,-b <format>: use binary format" << std::endl;
    if( verbose ) { std::cerr << comma::csv::format::usage() << std::endl; }
    std::cerr << std::endl;
    std::cerr << "options: " << std::endl;
    std::cerr << "    --mode <mode>: path mode (default: " << default_mode << ")" << std::endl;
    std::cerr << "    --proximity <proximity>: a wayline is traversed as soon as current position is within proximity of the endpoint (default: " << default_proximity << ")" << std::endl;
    std::cerr << "    --past-endpoint: a wayline is traversed as soon as current position is past the endpoint (or proximity condition is met)" << std::endl;
    std::cerr << "    --format: show binary format of default input stream and exit" << std::endl;
    std::cerr << "    --output-format: show binary format of output stream and exit" << std::endl;
    std::cerr << "    --output-fields: show output fields and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "path modes: " << std::endl;
    std::cerr << "    fixed: wait until the current waypoint is reached before accepting a new waypoint (first feedback position is the start of the first wayline)" << std::endl;
    std::cerr << "    dynamic: use a new waypoint as soon as it becomes available to define a new wayline form the current position to the new waypoint" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples: " << std::endl;
    std::cerr << "    cat targets.bin | " << name() << " \"tcp:localhost:12345;fields=t,x,y,,,,yaw;binary=t,6d\" --fields=x,y,,,speed --binary=3d,ui,d --past-endpoint > control.bin" << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

struct reached_t
{
    bool state;
    std::string reason;
    reached_t() : state( false ) {}
    reached_t( const std::string& reason ) : state( true ), reason( reason ) {}
    operator bool() const { return state; }
};

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options input_csv( options );
        comma::csv::input_stream< snark::control::target_t > input_stream( std::cin, input_csv );
        comma::csv::options output_csv( options );
        output_csv.full_xpath = true;
        output_csv.fields = field_names< snark::control::control_data_t >( true );
        if( input_csv.binary() ) { output_csv.format( format< snark::control::control_data_t >( output_csv.fields, true ) ); }
        comma::csv::output_stream< snark::control::control_data_t > output_stream( std::cout, output_csv );
        if( options.exists( "--format" ) ) { std::cout << format< snark::control::target_t >( input_csv.fields ) << std::endl; return 0; }
        if( options.exists( "--output-format" ) ) { std::cout << format< snark::control::control_data_t >( output_csv.fields, true ) << std::endl; return 0; }
        if( options.exists( "--output-fields" ) ) { std::cout << output_csv.fields << std::endl; return 0; }
        double proximity = options.value< double >( "--proximity", default_proximity );
        if( proximity <= 0 ) { std::cerr << name() << ": expected positive proximity, got " << proximity << std::endl; return 1; }
        snark::control::mode mode = snark::control::mode_from_string( options.value< std::string >( "--mode", default_mode ) );
        bool use_past_endpoint = options.exists( "--past-endpoint" );
        bool verbose = options.exists( "--verbose,-v" );
        std::vector< std::string > unnamed = options.unnamed( "--help,-h,--verbose,-v,--format,--output-format,--past-endpoint", "-.*,--.*" );
        if( unnamed.empty() ) { std::cerr << name() << ": feedback stream is not given" << std::endl; return 1; }
        comma::csv::options feedback_csv = comma::name_value::parser( "filename", ';', '=', false ).get< comma::csv::options >( unnamed[0] );
        if( feedback_csv.fields.empty() ) { feedback_csv.fields = field_names< snark::control::feedback_t >(); }
        comma::io::istream feedback_in( feedback_csv.filename, feedback_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii, comma::io::mode::non_blocking );
        comma::csv::input_stream< snark::control::feedback_t > feedback_stream( *feedback_in, feedback_csv );
        comma::io::select select;
        select.read().add( feedback_in );
        boost::optional< snark::control::vector_t > from;
        snark::control::wayline_t wayline;
        comma::signal_flag is_shutdown;
        while( !is_shutdown && ( input_stream.ready() || ( std::cin.good() && !std::cin.eof() ) ) )
        {
            reached_t reached;
            const snark::control::target_t* target = input_stream.read();
            if( !target ) { break; }
            snark::control::vector_t to = target->location;
            double heading_offset = target->parameters.heading_offset;
            if( verbose ) { std::cerr << name() << ": received target waypoint " << snark::control::serialise( to ) << std::endl; }
            if( from && snark::control::distance( *from, to ) < proximity ) { continue; }
            if( from ) { wayline = snark::control::wayline_t( *from, to, verbose ); }
            while( !is_shutdown && std::cout.good() )
            {
                if( mode == snark::control::dynamic && input_stream.ready() ) { from = boost::none; break; }
                select.check();
                if( feedback_stream.ready() || select.read().ready( feedback_in ) )
                {
                    const snark::control::feedback_t* feedback = feedback_stream.read();
                    if( !feedback ) { std::cerr << name() << ": feedback stream error occurred prior to reaching waypoint " << snark::control::serialise( to ) << std::endl; return 1; }
                    snark::control::position_t current = feedback->data;
                    if( !from )
                    {
                        from = current.location;
                        wayline = snark::control::wayline_t( *from, to, verbose );
                    }
                    if( snark::control::distance( current.location, to ) < proximity ) { reached = reached_t( "proximity" ); }
                    if( use_past_endpoint && wayline.is_past_endpoint( current.location ) ) { reached = reached_t( "past endpoint" ); }
                    if( reached )
                    {
                        if( verbose ) { std::cerr << name() << ": waypoint " << snark::control::serialise( to ) << " is reached (" << reached.reason << ")" << std::endl; }
                        if( mode == snark::control::fixed ) { from = to; }
                        else if( mode == snark::control::dynamic ) { from = boost::none; }
                        else { COMMA_THROW( comma::exception, "received unrecongnised mode " << snark::control::mode_to_string( mode ) ); }
                        break;
                    }
                    snark::control::error_t error;
                    error.cross_track = wayline.cross_track_error( current.location );
                    error.heading = wayline.heading_error( current.orientation.yaw, heading_offset );
                    output_stream.write( snark::control::control_data_t( *target, wayline, *feedback, error ) );
                }
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << name() << ": unknown exception" << std::endl; }
    return 1;
}
