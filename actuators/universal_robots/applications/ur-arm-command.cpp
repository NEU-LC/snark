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

#include <stdint.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/thread.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/base/types.h>
#include <comma/math/compare.h>
#include <comma/visiting/apply.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include <comma/io/publisher.h>
#include <comma/string/string.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/traits.h>
#include "../arm.h"
#include "../config.h"

static const char* name() { return "ur-arm-command"; }

void usage()
{
    std::cerr << std::endl;
    std::cerr << "take six comma-separated joint angles and output ur-script-formatted commands to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "commands:" << std::endl;
    std::cerr << "    init: move joints for the duration of time (use for initialisation); requires speed, acceleration, and time" << std::endl;
    std::cerr << "    stop: stop joint movement; requires acceleration" << std::endl;
    std::cerr << "    move: move joints until joint angles are equal to values read from input" << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h: show this message" << std::endl;
//    std::cerr << "    --config: path to config file" << std::endl;
    std::cerr << "    --acceleration: angular acceleration of the leading axis" << std::endl;
    std::cerr << "    --speed: angular speed of the leading axis" << std::endl;
    std::cerr << "    --radius: blend radius" << std::endl;
    std::cerr << "    --time: time to execute the motion" << std::endl;    
    std::cerr << "examples:" << std::endl;
    std::cerr << "    echo \"0,0,0,3.14,0,0\" | ur-arm-command move | nc robot-arm 30002" << std::endl;
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

struct move_options_t
{
    typedef double type;
    move_options_t( const comma::command_line_options& options )
    {
        acceleration = options.optional< type >( "--acceleration" );
        speed = options.optional< type >( "--speed" );
        time = options.optional< type >( "--time" );
        radius = options.optional< type >( "--radius" );
        std::stringstream ss;
        if( acceleration ) ss << ",a=" << *acceleration;
        if( speed ) ss << ",v=" << *speed;
        if( time ) ss << ",t=" << *time;
        if( radius ) ss << ",r=" << *radius;
        optional_arguments = ss.str();
    }
    std::string operator()() { return optional_arguments; };
    std::string optional_arguments;
    boost::optional< type > acceleration;
    boost::optional< type > speed;
    boost::optional< type > time;
    boost::optional< type > radius;
};

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "-h,--help" ) ) { usage(); }
        comma::csv::options csv = comma::csv::options( options );
        const std::vector< std::string > unnamed = options.unnamed( "--help,-h", "-.*,--.*" );
        if( unnamed.size() != 1 ) { std::cerr << name() << ": expected one command, got " << unnamed.size() << std::endl; return 1; }
        const std::string command = unnamed[0];
        //boost::optional< comma::ur::config_t > config;
        //if( options.exists( "--config" ) ) { config.reset( options.value< std::string >( "--config" ) ); }
        //if( config ) { std::cerr << config->move_options.acceleration << std::endl; }
        move_options_t m( options );
        if( command == "stop" ) 
        {
            if( !m.acceleration ) { std::cerr << name() << ": acceleration is not given" << std::endl; return 1; } 
            std::cout << "stopj(" << *m.acceleration << ")" << std::endl; 
            return 0;
        }
        else if( command == "init" ) 
        { 
            if( !m.acceleration ) { std::cerr << name() << ": acceleration is not given" << std::endl; return 1; } 
            if( !m.speed ) { std::cerr << name() << ": speed is not given" << std::endl; return 1; } 
            if( !m.time ) { std::cerr << name() << ": time is not given" << std::endl; return 1; } 
            std::vector< double > s( comma::ur::number_of_joints, *m.speed );
            std::cout << "speedj_init([" << comma::join( s, ',' ) << "]" << "," << *m.acceleration << "," << *m.time << ")" << std::endl;
            
            return 0;
        }
        else if( command == "move" ) 
        {
            comma::csv::input_stream< input_t > istream( std::cin, csv );
            comma::signal_flag is_shutdown;
            while( !is_shutdown && std::cin.good() && !std::cin.eof() )
            {
                const input_t* input = istream.read();
                if( !input ) { break; }
                std::cout << "movej([" << comma::join( input->values, ',' ) << "]" << m() << ")" << std::endl;
            }
        }
        else
        { 
            std::cerr << name() << ": expected a command, got" << command << std::endl; return 0; 
        }
    }
    catch( std::exception& ex ) { std::cerr << name() << ": " << ex.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception" << std::endl; return 1; }
}
