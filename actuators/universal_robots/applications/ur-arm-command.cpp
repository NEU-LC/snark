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
    std::cerr << "    --radius: blend radius" << std::endl;
    std::cerr << "    --time: time to execute the motion" << std::endl;
    if( verbose ) { std::cerr << "csv options" << std::endl << comma::csv::options::usage( "values" ) << std::endl; }
    else { std::cerr << "csv options... use --help --verbose for more" << std::endl << std::endl; }
    std::cerr << "examples (assuming robot.arm is the arm's IP address):" << std::endl;
    std::cerr << "    attempt to initialise one joint:" << std::endl;
    std::cerr << "        ur-arm-command init --speed=0,0,-0.05,0,0,0 --time=15 --acceleration=0.1 | nc robot.arm 30002" << std::endl;
    std::cerr << "    change joint angles at a speed of 0.02 rad/sec to new values 0,0,0,3.14,0,0:" << std::endl;
    std::cerr << "        echo \"3.14,0,0,0,0,0\" | ur-arm-command move --speed=0.02 | nc robot.arm 30002" << std::endl;    
    std::cerr << "    change joint angles in 10 seconds to new values 0,0,0,3.14,0,0:" << std::endl;
    std::cerr << "        echo \"3.14,0,0,0,0,0\" | ur-arm-command move --time=10 | nc robot.arm 30002" << std::endl;
    std::cerr << "    same as above but with a timestamp in the input stream (the time stamp is ignored):" << std::endl;
    std::cerr << "        echo \"20140101T000000,3.14,0,0,0,0,0\" | ur-arm-command move --time=10 --fields=t,values | nc robot.arm 30002" << std::endl;
    std::cerr << "    same as above but with the first and second input values swapped:" << std::endl;
    std::cerr << "        echo \"3.14,0,0,0,0,0\" | ur-arm-command move --time=10 --fields=values[1],values[0],values[2],values[3],values[4],values[5] | nc robot.arm 30002" << std::endl;    
    std::cerr << std::endl;
    exit ( -1 );
}

static const unsigned int number_of_input_fields = comma::ur::number_of_joints;
struct input_t { boost::array< double, number_of_input_fields > values; };

namespace comma { namespace visiting {    
    
template <> struct traits< input_t >
{
    template< typename K, typename V > static void visit( const K&, input_t& t, V& v )
    {
        v.apply( "values", t.values );
    }
    template< typename K, typename V > static void visit( const K&, const input_t& t, V& v )
    {
        v.apply( "values", t.values );
    }
};
    
} } // namespace comma { namespace visiting {

class move_options_t
{
public:
    typedef double type;
    move_options_t() {};
    move_options_t( const comma::command_line_options& options )
        : acceleration_( options.optional< type >( "--acceleration" ) )
        , speed_( options.optional< type >( "--speed" ) )
        , time_( options.optional< type >( "--time" ) )
        , radius_( options.optional< type >( "--radius" ) )
    {
        std::stringstream ss;
        if( acceleration_ ) ss << ",a=" << *acceleration_;
        if( speed_ ) ss << ",v=" << *speed_;
        if( time_ ) ss << ",t=" << *time_;
        if( radius_ ) ss << ",r=" << *radius_;
        optional_parameters_ = ss.str();
    }
    const std::string& optional_parameters() const { return optional_parameters_; }
    
private:
    std::string optional_parameters_;
    boost::optional< type > acceleration_;
    boost::optional< type > speed_;
    boost::optional< type > time_;
    boost::optional< type > radius_;
};

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv = comma::csv::options( options );
        const std::vector< std::string > unnamed = options.unnamed( "--help,-h", "-.*,--.*" );
        if( unnamed.size() != 1 ) { std::cerr << name() << ": expected one command, got " << unnamed.size() << std::endl; return 1; }
        const std::string command = unnamed[0];
        //boost::optional< comma::ur::config_t > config;
        //if( options.exists( "--config" ) ) { config = comma::read_json< comma::ur::config_t >( options.value< std::string >( "--config" ) ); }
        //if( config ) { std::cerr << config->move_options.acceleration << std::endl; }
        if( command == "init" )
        {
            if( !options.exists( "--speed" ) ||!options.exists( "--acceleration" ) ||  !options.exists( "--time" ) ) { std::cerr << name() << ": --acceleration, --speed, and --time are required for init" << std::endl; return 1; }
            std::vector< std::string > s = comma::split( options.value< std::string >( "--speed" ), ',' );
            if( s.size() != 1 && s.size() != comma::ur::number_of_joints ) { std::cerr << name() << ": expected 1 or " << comma::ur::number_of_joints << " comma-separated speed values, got: " << s.size() << std::endl; return 1; }
            std::vector< double > speed( comma::ur::number_of_joints );
            for( std::size_t i = 0; i < comma::ur::number_of_joints; ++i ) { speed[i] = boost::lexical_cast< double >( s[s.size() == 1 ? 0 : i] ); }
            double acceleration = options.value< double >( "--acceleration" ); 
            double time = options.value< double >( "--time" );
            std::cout << "speedj_init([" << comma::join( speed, ',' ) << "]" << "," << acceleration << "," << time << ")" << std::endl;
            return 0;
        }
        else if( command == "move" ) 
        {
            move_options_t move_options( options );
            comma::csv::input_stream< input_t > istream( std::cin, csv );
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const input_t* input = istream.read();
                if( !input ) { break; }
                std::cout << "movej([" << comma::join( input->values, ',' ) << "]" << move_options.optional_parameters() << ")" << std::endl;
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
