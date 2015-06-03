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

enum control_mode_t { skid, omni };
typedef boost::bimap< control_mode_t, std::string > named_control_mode_t;
static const named_control_mode_t named_control_modes = boost::assign::list_of< named_control_mode_t::relation >
    ( skid, "skid" )
    ( omni, "omni" );
control_mode_t control_mode_from_string( std::string s )
{
    if( !named_control_modes.right.count( s ) ) { COMMA_THROW( comma::exception, "control mode '" << s << "' is not found" ); }; 
    return  named_control_modes.right.at( s );
}
std::string control_mode_to_string( control_mode_t m ) { return  named_control_modes.left.at( m ); }

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "take control data on stdin and output command to stdout with the appended command" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: " << name << " [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --control-mode <mode>: control mode (available modes: skid, omni)" << std::endl;
    std::cerr << "    --cross-track-pid=<p>,<i>,<d>[,<error threshold>]: cross track pid parameters" << std::endl;
    std::cerr << "    --heading-pid=<p>,<i>,<d>[,<error threshold>]: heading pid parameters" << std::endl;
    std::cerr << "    --format: show default binary format of input stream and exit" << std::endl;
    std::cerr << "    --output-format: show binary format of output stream and exit" << std::endl;
    std::cerr << "    --output-fields: show output fields and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "control modes:" << std::endl;
    std::cerr << "    skid: skid-steer mode" << std::endl;
    std::cerr << "    omni: omnidirectional mode" << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr << comma::csv::options::usage() << std::endl;
        std::cerr << "default input fields:" << std::endl;
        std::cerr << field_names< control_data_t >( true, '\n' ) << std::endl;
        std::cerr << std::endl;
    }
    std::cerr << "examples:" << std::endl;
    std::cerr << "    cat targets.csv | " << snark::control::error_app_name << " \"feedback\" | " << name << " --cross-track-pid=0.1,0,0 --heading-pid=0.2,0,0 --control-mode=omni" << std::endl;
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
        control_mode_t control_mode = control_mode_from_string( options.value< std::string >( "--control-mode" ) );
        comma::csv::options input_csv( options, field_names< control_data_t >( true ) );
        input_csv.full_xpath = true;
        bool feedback_has_time  = input_csv.has_field( "feedback/t" );
        comma::csv::input_stream< control_data_t > input_stream( std::cin, input_csv );
        comma::csv::options output_csv( options );
        if( control_mode == omni ) { output_csv.fields = "turn_rate,local_heading"; }
        else if( control_mode == skid ) { output_csv.fields = "turn_rate"; }
        else { std::cerr << name << ": control mode " << control_mode_to_string( control_mode ) << "is not implemented" << std::endl; return 1; }
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
            if( control_mode == omni )
            {
                double heading = control_data->wayline.get_heading();
                double heading_correction = limit_angle( cross_track_pid.update( control_data->error.cross_track, time ) );
                double yaw = control_data->feedback.data.orientation.yaw;
                command.local_heading = snark::control::angle_wrap( heading + heading_correction - yaw );
                command.turn_rate = heading_pid.update( control_data->error.heading, time );
            }
            else if( control_mode == skid )
            {
                double heading_correction = limit_angle( cross_track_pid.update( control_data->error.cross_track, time ) );
                command.turn_rate = heading_pid.update( control_data->error.heading + heading_correction, time );
            }
            else
            {
                std::cerr << name << ": control mode " << control_mode_to_string( control_mode ) << "is not implemented" << std::endl;
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