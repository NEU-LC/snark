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

#include <cmath>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <comma/application/signal_flag.h>
#include "control.h"
#include "pid.h"

static const std::string name = snark::control::command_app_name;

template< typename T > std::string field_names( bool full_xpath = false, char separator = ',' ) { return comma::join( comma::csv::names< T >( full_xpath ), separator ); }
template< typename T > std::string format( std::string fields, bool full_xpath = false ) { return comma::csv::format::value< T >( fields, full_xpath ); }

typedef snark::control::control_data_t control_data_t;
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
    std::cerr << "take control data on stdin and output command to stdout with the appended command" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: " << name << " [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --steering <mode>: steering mode" << std::endl;
    std::cerr << "    --cross-track-pid=<p>,<i>,<d>[,<integral threshold>]: cross track pid parameters" << std::endl;
    std::cerr << "    --heading-pid=<p>,<i>,<d>[,<integral threshold>]: heading pid parameters" << std::endl;
    std::cerr << "    --format: show default binary format of input stream and exit" << std::endl;
    std::cerr << "    --output-format: show binary format of output stream and exit" << std::endl;
    std::cerr << "    --output-fields: show output fields and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "steering modes:" << std::endl;
    std::cerr << "    skid: skid-steer mode" << std::endl;
    std::cerr << "    omni: omnidirectional mode" << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr << comma::csv::options::usage() << std::endl;
        std::cerr << "default input fields:" << std::endl;
        std::vector< std::string > v = comma::split( field_names< control_data_t >( true ), ',' );
        for( std::vector< std::string >::const_iterator it = v.begin(); it != v.end(); ++it ) { std::cerr  << "    " << *it << std::endl; }
        std::cerr << std::endl;
    }
    std::cerr << "examples:" << std::endl;
    std::cerr << "    cat targets.csv | " << snark::control::error_app_name << " \"feedback\" --mode=fixed | " << name << " --cross-track-pid=0.1,0,0 --heading-pid=0.2,0,0 --steering=omni" << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

double limit_angle( double angle, double limit = M_PI/2 )
{
    if( angle > limit ) { return limit; }
    else if( angle < -limit ) { return -limit; }
    else { return angle; }
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        steering_t steering = steering_from_string( options.value< std::string >( "--steering" ) );
        comma::csv::options input_csv( options, field_names< control_data_t >( true ) );
        input_csv.full_xpath = true;
        bool feedback_has_time  = input_csv.has_field( "feedback/t" );
        comma::csv::input_stream< control_data_t > input_stream( std::cin, input_csv );
        comma::csv::options output_csv( options );
        if( steering == omni ) { output_csv.fields = "turn_rate,local_heading"; }
        else if( steering == skid ) { output_csv.fields = "turn_rate"; }
        else { std::cerr << name << ": steering '" << steering_to_string( steering ) << "' is not implemented" << std::endl; return 1; }
        if( input_csv.binary() ) { output_csv.format( format< command_t >( output_csv.fields ) ); }
        comma::csv::output_stream< command_t > output_stream( std::cout, output_csv );
        comma::csv::tied< control_data_t, command_t > tied( input_stream, output_stream );
        if( options.exists( "--format" ) ) { std::cout << format< control_data_t >( input_csv.fields, true ) << std::endl; return 0; }
        if( options.exists( "--output-format" ) ) { std::cout << format< control_data_t >( input_csv.fields, true ) << ',' << format< command_t >( output_csv.fields ) << std::endl; return 0; }
        if( options.exists( "--output-fields" ) ) { std::cout << input_csv.fields << ',' << output_csv.fields << std::endl; return 0; }
        snark::control::pid<> cross_track_pid( options.value< std::string >( "--cross-track-pid" ) );
        snark::control::pid<> heading_pid( options.value< std::string >( "--heading-pid" ) );
        comma::signal_flag is_shutdown;
        while( !is_shutdown && ( input_stream.ready() || ( std::cin.good() && !std::cin.eof() ) ) )
        {
            const control_data_t* control_data = input_stream.read();
            if( !control_data ) { break; }
            boost::posix_time::ptime time = feedback_has_time ? control_data->feedback.t : boost::posix_time::microsec_clock::universal_time();
            command_t command;
            if( steering == omni )
            {
                double heading = control_data->wayline.get_heading();
                double local_heading_correction = limit_angle( cross_track_pid.update( control_data->error.cross_track, time ) );
                double yaw = control_data->feedback.yaw;
                command.local_heading = snark::control::wrap_angle( yaw - heading + local_heading_correction );
                command.turn_rate = heading_pid.update( control_data->error.heading, time );
            }
            else if( steering == skid )
            {
                double heading_error_correction = limit_angle( cross_track_pid.update( control_data->error.cross_track, time ) );
                command.turn_rate = heading_pid.update( control_data->error.heading + heading_error_correction, time );
            }
            else
            {
                std::cerr << name << ": steering '" << steering_to_string( steering ) << "' is not implemented" << std::endl;
                return 1;
            }
            tied.append( command );
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << name << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << name << ": unknown exception" << std::endl; }
    return 1;
}