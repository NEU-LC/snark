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

#include <boost/bimap.hpp>
#include <boost/assign.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/csv/names.h>
#include <comma/io/select.h>
#include <comma/math/cyclic.h>
#include "../pid.h"
#include "control.h"
#include "traits.h"

static const std::string name = "control-command";

template< typename T > std::string field_names( bool full_xpath = false ) { return comma::join( comma::csv::names< T >( full_xpath ), ',' ); }
template< typename T > std::string format( const std::string& fields = "", bool full_xpath = false ) { return comma::csv::format::value< T >( !fields.empty() ? fields : field_names< T >( full_xpath ), full_xpath ); }
template< typename T > std::string format( bool full_xpath = false ) { return format< T >( "", full_xpath ); }

typedef snark::control::control_command_input_t input_t;
typedef snark::control::command_t command_t;

enum steering_t { skid, omni };
typedef boost::bimap< steering_t, std::string > named_steering_t;
static const named_steering_t named_steerings = boost::assign::list_of< named_steering_t::relation >
    ( skid, "skid" )
    ( omni, "omni" );
steering_t steering_from_string( std::string s )
{
    if( !named_steerings.right.count( s ) ) { COMMA_THROW( comma::exception, "steering '" << s << "' is not found" ); };
    return  named_steerings.right.at( s );
}
std::string steering_to_string( steering_t m ) { return  named_steerings.left.at( m ); }

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "take control data on stdin and output it to stdout with the appended command" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: " << name << " [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --steering,-s=<mode>: steering mode" << std::endl;
    std::cerr << "    --cross-track-pid=<p>,<i>,<d>[,<integral threshold>]: cross track pid parameters" << std::endl;
    std::cerr << "    --heading-pid=<p>,<i>,<d>[,<integral threshold>]: heading pid parameters" << std::endl;
    std::cerr << "    --reset: pid's are reset every time target waypoint changes" << std::endl;
    std::cerr << "    --input-fields: show default input stream fields and exit" << std::endl;
    std::cerr << "    --format,--input-format: show binary format of default input stream fields and exit" << std::endl;
    std::cerr << "    --output-format: show binary format of output stream and exit (for command fields only)" << std::endl;
    std::cerr << "    --output-fields: show output fields and exit (for command fields only)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "steering modes:" << std::endl;
    std::cerr << "    skid: skid-steer mode" << std::endl;
    std::cerr << "    omni: omnidirectional mode" << std::endl;
    std::cerr << std::endl;
    std::cerr << "note:" << std::endl;
    std::cerr << "    if --fields has 'feedback/yaw_rate', the rate provided is used in computing heading pid" << std::endl;
    std::cerr << "    otherwise rate is computed internally, provided that --fields has 'feedback/t'" << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr << comma::csv::options::usage() << std::endl;
        std::cerr << "default input fields:" << std::endl;
        std::vector< std::string > v = comma::split( field_names< input_t >( true ), ',' );
        for( std::vector< std::string >::const_iterator it = v.begin(); it != v.end(); ++it ) { std::cerr  << "    " << *it << std::endl; }
        std::cerr << std::endl;
    }
    std::cerr << "examples:" << std::endl;
    std::cerr << "    cat targets.csv | control-error \"feedback\" --mode=fixed | " << name << " --cross-track-pid=0.1,0,0 --heading-pid=0.2,0,0 --steering=omni" << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

static double limit_angle( double angle, double limit = M_PI/2 )
{
    if( angle > limit ) { return limit; }
    else if( angle < -limit ) { return -limit; }
    else { return angle; }
}

template < typename PID >
static PID make_pid( const std::string& pid_values, char delimiter = ',' )
{
    std::vector< std::string > v = comma::split( pid_values, delimiter );
    if( v.size() != 3 && v.size() != 4 ) { COMMA_THROW( comma::exception, "expected a string with 3 or 4 elements separated by '" << delimiter << "', got " << v.size() ); }
    double p = boost::lexical_cast< double >( v[0] );
    double i = boost::lexical_cast< double >( v[1] );
    double d = boost::lexical_cast< double >( v[2] );
    if( v.size() == 4 )
    {
        double threshold = boost::lexical_cast< double >( v[3] );
        if( threshold <= 0 ) { COMMA_THROW( comma::exception, "expected positive threshold, got " << threshold ); }
        return PID( p, i, d, threshold );
    }
    else
    {
        return PID( p, i, d );
    }
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options input_csv( options );
        input_csv.full_xpath = true;
        comma::csv::input_stream< input_t > input_stream( std::cin, input_csv );
        comma::csv::options output_csv( options );
        if( options.exists( "--format,--input-format" ) ) { std::cout << format< input_t >( true ) << std::endl; return 0; }
        if( options.exists( "--input-fields" ) ) { std::cout << field_names< input_t >( true ) << std::endl; return 0; }
        steering_t steering = steering_from_string( options.value< std::string >( "--steering,-s" ) );
        if( steering == omni ) { output_csv.fields = "turn_rate,local_heading"; }
        else if( steering == skid ) { output_csv.fields = "turn_rate"; }
        else { std::cerr << name << ": steering '" << steering_to_string( steering ) << "' is not implemented" << std::endl; return 1; }
        if( input_csv.binary() ) { output_csv.format( format< command_t >( output_csv.fields ) ); }
        comma::csv::output_stream< command_t > output_stream( std::cout, output_csv );
        comma::csv::tied< input_t, command_t > tied( input_stream, output_stream );
        if( options.exists( "--output-format" ) ) { std::cout << format< command_t >( output_csv.fields ) << std::endl; return 0; }
        if( options.exists( "--output-fields" ) ) { std::cout << output_csv.fields << std::endl; return 0; }
        bool feedback_has_time  = input_csv.has_field( "feedback/t" );
        bool compute_yaw_rate = !input_csv.has_field( "feedback/yaw_rate" );
        bool reset_pid = options.exists( "--reset" );
        snark::control::pid cross_track_pid = make_pid< snark::control::pid >( options.value< std::string >( "--cross-track-pid" ) );
        snark::control::pid heading_pid = make_pid< snark::control::angular_pid >( options.value< std::string >( "--heading-pid" ) );
        boost::optional< snark::control::wayline::position_t > position;
        boost::optional< snark::control::wayline::position_t > previous_position;
        comma::signal_flag is_shutdown;
        while( !is_shutdown && ( input_stream.ready() || ( std::cin.good() && !std::cin.eof() ) ) )
        {
            const input_t* input = input_stream.read();
            if( !input ) { break; }
            if( reset_pid )
            {
                if( position ) { previous_position = position; }
                position = input->target.position;
                if( previous_position && !position->isApprox( *previous_position ) )
                {
                    cross_track_pid.reset();
                    heading_pid.reset();
                }
            }
            command_t command;
            boost::posix_time::ptime time = feedback_has_time ? input->feedback.t : boost::posix_time::not_a_date_time;
            switch( steering )
            {
                case omni:
                {
                    double correction = limit_angle( cross_track_pid( input->error.cross_track, time ) );
                    command.local_heading = comma::math::cyclic< double >( comma::math::interval< double >( -M_PI, M_PI ), input->wayline.heading + correction - input->feedback.yaw )();
                    command.turn_rate = compute_yaw_rate ? heading_pid( input->error.heading, time ) : heading_pid( input->error.heading, input->feedback.yaw_rate, time );
                    break;
                }
                case skid:
                {
                    double error = input->error.heading - limit_angle( cross_track_pid( input->error.cross_track, time ) );
                    command.turn_rate = compute_yaw_rate ? heading_pid( error, time ) : heading_pid( error, input->feedback.yaw_rate, time );
                    break;
                }
            }
            tied.append( command );
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << name << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << name << ": unknown exception" << std::endl; }
    return 1;
}