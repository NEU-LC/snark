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

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/csv/traits.h>
#include <comma/name_value/serialize.h>
#include "base.h"
#include "config.h"

static const char* name() { return "ur-arm-command"; }

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "take six comma-separated joint angles and output ur-script-formatted commands to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "commands:" << std::endl;
    std::cerr << "    init: move joints for the duration of time (used for initialisation); requires speed, acceleration, and time" << std::endl;
    std::cerr << "    move: move joints until joint angles are equal to values read from input" << std::endl;
    std::cerr << "    stop: stop joint movement; requires acceleration" << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h: show this message; --help --verbose: more help" << std::endl;
//    std::cerr << "    --config: path to config file" << std::endl;
    std::cerr << "    --acceleration: angular acceleration of the leading axis" << std::endl;
    std::cerr << "    --speed: angular speed of the leading axis" << std::endl;
    std::cerr << "    --time: time to execute the motion" << std::endl;
    std::cerr << "    --radius: blend radius" << std::endl;
    std::cerr << "    --fields: input stream fields (default: values, optional: acceleration,speed,time,radius)" << std::endl;
    std::cerr << "        note: input values of optional fields take priority over values given via options on the command line" << std::endl;
    if( verbose ) { std::cerr << "csv options" << std::endl << comma::csv::options::usage( "angles" ) << std::endl; }
    else { std::cerr << "csv options... use --help --verbose for more" << std::endl << std::endl; }
    std::cerr << "examples (assuming robot.arm is the arm's IP address):" << std::endl;
    std::cerr << "    change joint angles at a speed of 0.02 rad/sec to new values 0,0,0,3.14,0,0:" << std::endl;
    std::cerr << "        echo \"3.14,0,0,0,0,0\" | ur-arm-command move --speed=0.02 | nc robot.arm 30002" << std::endl;    
    std::cerr << "    change joint angles in 10 seconds to new values 0,0,0,3.14,0,0:" << std::endl;
    std::cerr << "        echo \"3.14,0,0,0,0,0\" | ur-arm-command move --time=10 | nc robot.arm 30002" << std::endl;
    std::cerr << "    same as above but with a timestamp in the input stream (the time stamp is ignored):" << std::endl;
    std::cerr << "        echo \"20140101T000000,3.14,0,0,0,0,0\" | ur-arm-command move --time=10 --fields=t,angles | nc robot.arm 30002" << std::endl;
    std::cerr << "    same as above but with the first and second input angles swapped:" << std::endl;
    std::cerr << "        echo \"3.14,0,0,0,0,0\" | ur-arm-command move --time=10 --fields=angles[1],angles[0],angles[2],angles[3],angles[4],angles[5] | nc robot.arm 30002" << std::endl;    
    std::cerr << std::endl;
    exit ( -1 );
}

static const unsigned int number_of_input_fields = comma::ur::number_of_joints;
struct input_t 
{ 
    boost::array< double, number_of_input_fields > angles; 
    double acceleration;
    double speed;
    double time;
    double radius;
};

namespace comma { namespace visiting {    
    
template <> struct traits< input_t >
{
    template< typename K, typename V > static void visit( const K&, input_t& t, V& v )
    {
        v.apply( "angles", t.angles );
        v.apply( "acceleration", t.acceleration );
        v.apply( "speed", t.speed );
        v.apply( "time", t.time );
        v.apply( "radius", t.radius );
    }
    template< typename K, typename V > static void visit( const K&, const input_t& t, V& v )
    {
        v.apply( "angles", t.angles );
        v.apply( "acceleration", t.acceleration );
        v.apply( "speed", t.speed );
        v.apply( "time", t.time );
        v.apply( "radius", t.radius );
    }
};
    
} } // namespace comma { namespace visiting {

class optional_parameters_t
{
public:
    optional_parameters_t( const comma::command_line_options& options, const comma::csv::options& csv )
        : command_line_options_( options ), input_stream_has_( csv )
    {
        std::stringstream ss;
        if( command_line_options_.acceleration && !input_stream_has_.acceleration ) ss << ",a=" << *command_line_options_.acceleration;
        if( command_line_options_.speed && !input_stream_has_.speed ) ss << ",v=" << *command_line_options_.speed;
        if( command_line_options_.time && !input_stream_has_.time ) ss << ",t=" << *command_line_options_.time;
        if( command_line_options_.radius && !input_stream_has_.radius ) ss << ",r=" << *command_line_options_.radius;
        parameters_from_command_line_ = ss.str();
    }
    std::string operator()( const input_t* input )
    { 
        if( input_stream_has_.none ) { return parameters_from_command_line_; }
        else
        {
            std::stringstream ss;
            if( input_stream_has_.acceleration ) { ss << ",a=" << input->acceleration; }
            if( input_stream_has_.speed ) { ss << ",v=" << input->speed; }
            if( input_stream_has_.time ) { ss << ",t=" << input->time; }
            if( input_stream_has_.radius ) { ss << ",r=" << input->radius; }
            return parameters_from_command_line_ + ss.str();
        }
    }
    
private:
    struct command_line_options_t
    {
        command_line_options_t( const comma::command_line_options& options ) 
            : acceleration( options.optional< double >( "--acceleration" ) )
            , speed( options.optional< double >( "--speed" ) )
            , time( options.optional< double >( "--time" ) )
            , radius( options.optional< double >( "--radius" ) ) {};
        boost::optional< double > acceleration;
        boost::optional< double > speed;
        boost::optional< double > time;
        boost::optional< double > radius;
    };    
    struct input_stream_has_t
    {
        input_stream_has_t( const comma::csv::options& csv ) 
            : acceleration( csv.has_field( "acceleration" ) )
            , speed( csv.has_field( "speed" ) )
            , time( csv.has_field( "time" ) )
            , radius( csv.has_field( "radius" ) )
            , none( !acceleration && !speed && !time && !radius ) {};
        bool acceleration;
        bool speed;
        bool time;
        bool radius;
        bool none;
    };    
    command_line_options_t command_line_options_;
    input_stream_has_t input_stream_has_;
    std::string parameters_from_command_line_;
};

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv = comma::csv::options( options, "angles" );
        if( !( csv.has_field( "angles" ) || csv.has_field( "angles[0],angles[1],angles[2],angles[3],angles[4],angles[5]" ) ) ) { std::cerr << name() << ": expected input fields for angles of all joints, some are missing" << std::endl; return 1; }
        const std::vector< std::string > unnamed = options.unnamed( "--help,-h", "-.*,--.*" );
        if( unnamed.size() != 1 ) { std::cerr << name() << ": expected one command, got " << unnamed.size() << std::endl; return 1; }
        const std::string command = unnamed[0];
        //boost::optional< comma::ur::config_t > config;
        //if( options.exists( "--config" ) ) { config = comma::read_json< comma::ur::config_t >( options.value< std::string >( "--config" ) ); }
        //if( config ) { std::cerr << config->move_options.acceleration << std::endl; }
        if( command == "move" )
        {
            optional_parameters_t optional_parameters( options, csv );
            comma::csv::input_stream< input_t > istream( std::cin, csv );
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const input_t* input = istream.read();
                if( !input ) { break; }
                std::cout << "movej([" << comma::join( input->angles, ',' ) << "]" << optional_parameters( input ) << ")" << std::endl;
            }
            return 0;
        }
        else if( command == "stop" )
        {
            double acceleration = options.value< double >( "--acceleration", 0 );
            std::cout << "stopj(" << acceleration << ")" << std::endl;
            return 0;
        }
        else
        { 
            std::cerr << name() << ": expected a command, got" << command << std::endl; return 0; 
        }
    }
    catch( std::exception& ex ) { std::cerr << name() << ": " << ex.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception" << std::endl; return 1; }
}
