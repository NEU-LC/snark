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
#include "../traits.h"
#include "../commands.h"
#include "../commands_handler.h"
#include "../inputs.h"
#include "../units.h"
#include "../data.h"

static const char* name() { return "ur-arm-command"; }

void usage()
{
    std::cerr << std::endl;
    std::cerr << "take ur arm command arguments on stdin (e.g., six joint angles) and output ur-script-formatted command to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h: show this message" << std::endl;
    std::cerr << "    --config: path to arm config file" << std::endl;
    std::cerr << "general commands:" << std::endl;
    std::cerr << "    power-on: " << std::endl;
    std::cerr << "    power-off: " << std::endl;
    std::cerr << "    init: " << std::endl;
    std::cerr << "    init-joint: " << std::endl;
    std::cerr << "    stop: " << std::endl;
    std::cerr << "movement commands:" << std::endl;
    std::cerr << "    move-joints: " << std::endl;
    std::cerr << "    move-tool: " << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    echo \"0,0,0,3.14,0,0\" | ur-arm-command move-joints --config=ur-arm.json | nc robot-arm 30002" << std::endl;
    std::cerr << std::endl;
    exit ( -1 );
}

typedef snark::ur::handlers::commands_handler commands_handler_t;
typedef boost::shared_ptr< commands_handler_t > commands_handler_shared;
static commands_handler_shared commands_handler;
static snark::ur::config_t config;
using snark::ur::handlers::result;

template < typename T >
std::string handle( const std::vector< std::string >& line, std::ostream& os )
{
    static comma::csv::ascii< T > ascii;
    T command;
    try { command = ascii.get( line ); }
    catch( boost::bad_lexical_cast& le ) { std::ostringstream ss; ss << comma::join( line, ',' ) << ": wrong field types, expected fields: " << command.names(); return ss.str(); }
    catch( comma::exception& ce ) { std::ostringstream ss; ss << comma::join( line, ',' ) << ": wrong fields or field types, expected fields: " << command.names(); return ss.str(); }
    catch( ... ) { COMMA_THROW( comma::exception, "unknown error is parsing: " + comma::join( line , ',' ) ); }       
    comma::dispatch::handler& h_ref( *commands_handler );
    comma::dispatch::dispatched_base& ref( command );
    ref.dispatch_to( h_ref );
    std::ostringstream ss; ss << command.serialise() << ": " << commands_handler->ret.get_message(); return ss.str();
}

void process_command( const std::vector< std::string >& v, std::ostream& os )
{
    if( boost::iequals( v[0], "power" ) )       { std::cout << handle< snark::ur::power >( v, os ) << std::endl; }  
    else if( boost::iequals( v[0], "brakes" ) || boost::iequals( v[0], "stop" ) ) { std::cout << handle< snark::ur::brakes >( v, os ) << std::endl; }  
    else if( boost::iequals( v[0], "cancel" ) ) { } /// No need to do anything, used to cancel other running commands e.g. auto_init
    else if( boost::iequals( v[0], "auto_init" ) ) { std::cout << handle< snark::ur::auto_init >( v, os ) << std::endl; }
    else if( boost::iequals( v[0], "initj" ) ) { std::cout << handle< snark::ur::joint_move >( v, os ) << std::endl; }
    else { std::cout << comma::join( v, v.size(), ',' ) << ":\"unknown command\"" << std::endl; return; }
}

void load_config( const std::string& filepath )
{
    std::ifstream config_ifs( filepath.c_str() );
    if( !config_ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open file: " + filepath ); }
    boost::property_tree::ptree t;
    boost::property_tree::read_json( config_ifs, t );
    comma::from_ptree from_ptree( t, true );
    comma::visiting::apply( from_ptree ).to( config );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "-h,--help" ) ) { usage(); }
        std::string config_file = options.value< std::string >( "--config" );
        load_config( config_file );
        snark::ur::inputs inputs;
        snark::ur::status_t status; 
        commands_handler.reset( new commands_handler_t( status, std::cout, config ) );
        comma::signal_flag is_shutdown;
        while( !is_shutdown && std::cin.good() && ! std::cin.eof() )
        {
            // TODO: get rid of inputs (want to read command argument on stdin, not the commands themselves)
            inputs.read(); // Also act as sleep, reads commands from stdin
            if( !inputs.is_empty() )
            {
                const std::vector< std::string > commands = inputs.front();
                inputs.pop();
                process_command( commands, std::cout );
            }
        }
    }
    catch( std::exception& ex ) { std::cerr << name() << ": " << ex.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception" << std::endl; return 1; }
}
