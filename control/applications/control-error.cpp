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
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/csv/names.h>
#include <comma/csv/traits.h>
#include <comma/io/select.h>
#include <comma/io/stream.h>
#include <comma/name_value/name_value.h>
#include "../control.h"
#include "../wrap_angle.h"
#include "../traits.h"

static const std::string name = "control-error";

template< typename T > std::string field_names( bool full_xpath = false ) { return comma::join( comma::csv::names< T >( full_xpath ), ',' ); }
template< typename T > std::string format( const std::string& fields = "", bool full_xpath = false ) { return comma::csv::format::value< T >( !fields.empty() ? fields : field_names< T >( full_xpath ), full_xpath ); }
static const double default_proximity = 0.1;
static const std::string default_mode = "fixed";

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "take target waypoints on stdin and feedback from address and output to stdout with the appended wayline heading and control errors" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: " << name << " <feedback> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "feedback: \"<address>[;<options>]\"" << std::endl;
    std::cerr << "    address: address where the feedback stream is published, e.g. tcp:localhost:12345" << std::endl;
    std::cerr << "    options: semicolon-separated csv options of the feedback stream, e.g. fields=t,x,y,,,,yaw;binary=t,6d" << std::endl;
    std::cerr << "    default fields: " << field_names< snark::control::feedback_t >() << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "csv stream options: " << std::endl;
        std::cerr << comma::csv::options::usage( field_names< snark::control::target_t >() ) << std::endl;
    }
    std::cerr << std::endl;
    std::cerr << "options: " << std::endl;
    std::cerr << "    --mode,-m=<mode>: control mode (default: " << default_mode << ")" << std::endl;
    std::cerr << "    --proximity,-p=<proximity>: a wayline is traversed as soon as current position is within proximity of the endpoint (default: " << default_proximity << ")" << std::endl;
    std::cerr << "    --past-endpoint: a wayline is traversed as soon as current position is past the endpoint (or proximity condition is met)" << std::endl;
    std::cerr << "    --frequency,-f=<frequency>: control frequency (the rate at which " << name <<" outputs control errors using latest feedback)" << std::endl;
    std::cerr << "    --heading-is-absolute: interpret heading offset as global heading by default" << std::endl;
    std::cerr << "    --input-fields: show default input stream fields and exit" << std::endl;
    std::cerr << "    --format,--input-format: show binary format of default input stream fields and exit" << std::endl;
    std::cerr << "    --output-format: show binary format of output stream and exit (for wayline and control error fields only)" << std::endl;
    std::cerr << "    --output-fields: show output fields and exit (for wayline and control error fields only)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "control modes: " << std::endl;
    std::cerr << "    fixed: wait until the current waypoint is reached before accepting a new waypoint (first feedback position is the start of the first wayline)" << std::endl;
    std::cerr << "    dynamic: use a new waypoint as soon as it becomes available to define a new wayline form the current position to the new waypoint" << std::endl;
    std::cerr << std::endl;
    std::cerr << "default input fields: position/x,position/y,heading_offset,is_absolute" << std::endl;
    std::cerr << "    required fields: position/x,position/y" << std::endl;
    std::cerr << "    optional fields: heading_offset,is_absolute (default values: 0,0)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples: " << std::endl;
    std::cerr << "    cat targets.bin | " << name << " \"tcp:localhost:12345;fields=t,x,y,,,,yaw;binary=t,6d\" --fields=x,y,,,speed --binary=3d,ui,d --past-endpoint" << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options input_csv( options, field_names< snark::control::target_t >() );
        const char delimiter = input_csv.delimiter;
        comma::csv::input_stream< snark::control::target_t > input_stream( std::cin, input_csv, snark::control::target_t( options.exists( "--heading-is-absolute" ) ) );
        comma::csv::options output_csv( options );
        output_csv.full_xpath = true;
        output_csv.fields = "wayline/heading,error/cross_track,error/heading,reached";
        if( input_csv.binary() ) { output_csv.format( format< snark::control::control_data_t >( output_csv.fields, true ) ); }
        comma::csv::output_stream< snark::control::control_data_t > output_stream( std::cout, output_csv );
        if( options.exists( "--input-fields" ) ) { std::cout << field_names< snark::control::target_t >( true ) << std::endl; return 0; }
        if( options.exists( "--format,--input-format" ) ) { std::cout << format< snark::control::target_t >() << std::endl; return 0; }
        if( options.exists( "--output-format" ) ) { std::cout << format< snark::control::control_data_t >( output_csv.fields, true ) << std::endl; return 0; }
        if( options.exists( "--output-fields" ) ) { std::cout << output_csv.fields << std::endl; return 0; }
        snark::control::mode_t mode = snark::control::mode_from_string( options.value< std::string >( "--mode", default_mode ) );
        bool use_delay = options.exists( "--frequency,-f" );
        boost::posix_time::microseconds delay( 0 );
        boost::posix_time::ptime next_output_time( boost::posix_time::microsec_clock::universal_time() );
        if( use_delay )
        {
            double frequency = options.value< double >( "--frequency,-f" );
            if( frequency <= 0 ) { std::cerr << name << ": expected positive frequency, got " << frequency << std::endl; return 1; }
            delay = boost::posix_time::microseconds( static_cast< long >( 1000000 / frequency ) );
        }
        bool verbose = options.exists( "--verbose,-v" );
        std::vector< std::string > unnamed = options.unnamed( "--help,-h,--verbose,-v,--input-fields,--format,--input-format,--output-format,--output-fields,--past-endpoint,--heading-is-absolute", "-.*,--.*" );
        if( unnamed.empty() ) { std::cerr << name << ": feedback stream is not given" << std::endl; return 1; }
        comma::csv::options feedback_csv = comma::name_value::parser( "filename", ';', '=', false ).get< comma::csv::options >( unnamed[0] );
        if( input_csv.binary() && !feedback_csv.binary() ) { std::cerr << name << ": cannot join binary input stream with ascii feedback stream" << std::endl; return 1; }
        if( !input_csv.binary() && feedback_csv.binary() ) { std::cerr << name << ": cannot join ascii input stream with binary feedback stream" << std::endl; return 1; }
        if( feedback_csv.fields.empty() ) { feedback_csv.fields = field_names< snark::control::feedback_t >(); }
        comma::io::istream feedback_in( feedback_csv.filename, feedback_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
        comma::csv::input_stream< snark::control::feedback_t > feedback_stream( *feedback_in, feedback_csv );
        comma::io::select select;
        select.read().add( feedback_in );
        boost::optional< snark::control::feedback_t > feedback;
        if( select.wait( boost::posix_time::seconds( 1 ) ) )
        {
            const snark::control::feedback_t* p = feedback_stream.read();
            if( !p ) { std::cerr << name << ": feedback stream error" << std::endl; return 1; }
            feedback = *p;
        }
        else
        {
            std::cerr << name << ": feedback is not publishing data" << std::endl;
            return 1;
        }
        select.read().add( comma::io::stdin_fd );
        std::deque< std::pair< snark::control::target_t, std::string > > targets;
        comma::signal_flag is_shutdown;
        bool first_target = true;
        snark::control::wayline_follower follower( mode, options.value< double >( "--proximity", default_proximity ), options.exists( "--past-endpoint" ) );
        while( !is_shutdown && std::cin.good() && std::cout.good() )
        {
            // todo? don't do select.check() on stdin in the loop or do it only in "dynamic" mode?
            while( !is_shutdown && ( input_stream.ready() || ( select.check() && select.read().ready( comma::io::stdin_fd ) ) ) )
            {
                const snark::control::target_t* p = input_stream.read();
                if( !p ) { break; }
                std::pair< snark::control::target_t, std::string > pair( *p, "" );
                if( input_csv.binary() )
                {
                    pair.second.resize( input_csv.format().size() );
                    ::memcpy( &pair.second[0], input_stream.binary().last(), pair.second.size() );
                }
                else
                {
                    pair.second = comma::join( input_stream.ascii().last(), input_csv.delimiter );
                }
                targets.push_back( pair );
            }
            while( !is_shutdown && ( feedback_stream.ready() || ( select.check() && select.read().ready( feedback_in ) ) ) )
            {
                const snark::control::feedback_t* p = feedback_stream.read();
                if( !p ) { std::cerr << name << ": end of feedback stream" << std::endl; return 1; }
                feedback = *p;
            }
            if( is_shutdown ) { break; }
            if( targets.empty() || !feedback ) { select.wait( boost::posix_time::millisec( 10 ) ); continue; }
            if( first_target || follower.reached_target() || ( mode == snark::control::dynamic && targets.size() > 1 ) )
            {
                if( mode == snark::control::dynamic )
                {
                    std::pair< snark::control::target_t, std::string > pair = targets.back();
                    targets.clear();
                    targets.push_back( pair );
                }
                follower.set_target( targets.front().first, feedback->position );
                first_target = false;
                if( verbose ) { std::cerr << name << ": target waypoint " << snark::control::serialise( follower.to() ) << std::endl; }
            }
            follower.update( *feedback );
            if( follower.reached_target() || ( use_delay && boost::posix_time::microsec_clock::universal_time() > next_output_time ) )
            {
                if( input_csv.binary() )
                {
                    std::cout.write( &targets.front().second[0], input_csv.format().size() );
                }
                else
                {
                    std::cout << targets.front().second << delimiter;
                }
                if( feedback_csv.binary() )
                {
                    std::cout.write( feedback_stream.binary().last(), feedback_csv.format().size() );
                }
                else
                {
                    std::cout << comma::join( feedback_stream.ascii().last(), delimiter ) << delimiter;
                }
                output_stream.write( snark::control::control_data_t( follower ) );
                if( use_delay ) { next_output_time = boost::posix_time::microsec_clock::universal_time() + delay; }
            }
            if( verbose && follower.reached_target() ) { std::cerr << name << ": reached waypoint " << snark::control::serialise( follower.to() ) << ", current position: " << snark::control::serialise( feedback->position ) << std::endl; }
            if( follower.reached_target() ) { targets.pop_front(); }
            feedback.reset();
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << name << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << name << ": unknown exception" << std::endl; }
    return 1;
}
