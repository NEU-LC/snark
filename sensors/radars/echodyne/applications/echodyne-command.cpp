// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2020 Mission Systems Pty Ltd
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright owner nor the names of the contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
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

#include <chrono>
#include <thread>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include "../radar.h"

const std::string default_address( "169.254.1.10" );
const int default_port( 23 );
const std::string default_log_dir( "/var/tmp" );

static void bash_completion( unsigned int const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " --autopause"
        ;

    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << "\nRead commands from stdin and send to a echodyne radar";
    std::cerr << "\n";
    std::cerr << "\nUsage: cat commands.csv | " << comma::verbose.app_name() << " [<options>]";
    std::cerr << "\n";
    std::cerr << "\nOptions:";
    std::cerr << "\n    --help,-h:       show this help";
    std::cerr << "\n    --verbose,-v:    more output to stderr";
    std::cerr << "\n    --address=<ip>:  device address; default=" << default_address;
    std::cerr << "\n    --port=<num>:    device port; default=" << default_port;
    std::cerr << "\n    --log-dir=<dir>: directory for system logs; default=" << default_log_dir;
    std::cerr << "\n    --autopause:     add a two second delay between commands";
    std::cerr << "\n    --stay:          do not close at end of input stream";
    std::cerr << "\n";
    std::cerr << "\nNote that responses are sent to stderr. This allows echodyne-cat to issue";
    std::cerr << "\ncommands without their response interfering with the data output.";
    std::cerr << "\n";
    std::cerr << "\nUser commands are defined in section 8 of the Echoflight User Manual";
    std::cerr << "\nParticularly useful commands include:";
    std::cerr << "\n    *IDN?  SYSPARAM?  *TST?  LIST        -  various info commands";
    std::cerr << "\n    ETH:IP? ETH:IP                       -  get or set ip address";
    std::cerr << "\n    SYS:TIME?                            -  get the device time";
    std::cerr << "\n    RESET:SYSTEM                         -  reboot device";
    std::cerr << "\n    MODE:SEARCH:START  MODE:SEARCH:STOP  -  search";
    std::cerr << "\n    MODE:SWT:START     MODE:SWT:STOP     -  search while tracking";
    std::cerr << "\n";
    std::cerr << "\nIn addition there are the following commands that use the C++ API:";
    std::cerr << "\n    API:BUFFERS                -  show the buffer states";
    std::cerr << "\n    API:ENABLE_BUFFER buffer   -  enable a buffer for capture";
    std::cerr << "\n    API:DISABLE_BUFFER buffer  -  disable a buffer for capture";
    std::cerr << "\n    API:SYS_STATE              -  show system state";
    std::cerr << "\n";
    std::cerr << "\nAnd the following complex commands:";
    std::cerr << "\n    CMD:SET_TIME               -  set the device time to system time";
    std::cerr << "\n";
    std::cerr << "\nExamples:";
    std::cerr << "\n    echo \"*IDN?\" | " << comma::verbose.app_name();
    std::cerr << "\n    echo \"CMD:SET_TIME\" | " << comma::verbose.app_name();
    std::cerr << "\n    echo \"ETH:IP?\" | " << comma::verbose.app_name();
    std::cerr << "\n    echo \"ETH:IP <new-ip> <mask> <gw>\" | " << comma::verbose.app_name() << " --address <curr-ip>";
    std::cerr << "\n    echo -e \"API:ENABLE_BUFFER STATUS\\nAPI:SYS_STATE\" | " << comma::verbose.app_name() << " --autopause";
    std::cerr << "\n    echo \"MODE:SWT:START\" | " << comma::verbose.app_name() << " --stay";
    std::cerr << "\n";
    std::cerr << std::endl;
    exit( 0 );
}

static std::unique_ptr< snark::echodyne::radar > radar;
static std::string input;
static comma::signal_flag is_shutdown;

int main( int argc, char** argv )
{
    using namespace std::chrono_literals;
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( argc, argv );
        std::string address = options.value< std::string >( "--address", default_address );
        int port = options.value< int >( "--port", default_port );
        std::string log_dir = options.value< std::string >( "--log-dir", default_log_dir );
        bool autopause = options.exists( "--autopause" );
        bool stay = options.exists( "--stay" );

        radar = std::make_unique< snark::echodyne::radar >( log_dir );
        radar->connect( address, port, log_dir );

        while( !is_shutdown && std::cin.good() )
        {
            std::getline( std::cin, input );
            if( autopause ) { std::this_thread::sleep_for( 2000ms ); }
            radar->command( input );
        }
        if( is_shutdown ) { std::cerr << comma::verbose.app_name() << ": interrupted by signal" << std::endl; }
        else if( stay ) { while( !is_shutdown ) { std::this_thread::sleep_for( 200ms ); } }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl;
    }
    return 1;
}
