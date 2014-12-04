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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/csv/traits.h>
#include <comma/application/signal_flag.h>
#include "base.h"
#include "mode.h"

static const char* name() { return "ur-arm-status"; }

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "take arm status feed on stdin, output csv data to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h: show this message" << std::endl;
    std::cerr << "    --verbose,-v: show more info (mode names, etc)" << std::endl;
    std::cerr << "    --binary,-b: output binary equivalent of csv" << std::endl;
    std::cerr << "    --format: output binary format for given fields to stdout and exit" << std::endl;
    std::cerr << "    --output-fields: output field names and exit" << std::endl;
    std::cerr << "    --packet-size: output packet-size and exit" << std::endl;
    std::cerr << "    --mode-from-name=<name>: output the integer corresponding to the given mode name and exit" << std::endl;
    std::cerr << "    --joint-mode-from-name=<name>: same as above for joint modes" << std::endl;
    std::cerr << "    --mode-to-name=<mode>: output the name of the given mode integer and exit" << std::endl;
    std::cerr << "    --joint-mode-to-name=<mode>: same as above for joint modes" << std::endl;    
    std::cerr << std::endl;
    std::cerr << "examples: " << std::endl;
    std::cerr << "    socat -u tcp:robot.arm:30003 - | " << name() << " --fields=t,mode,joints/mode" << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "modes: " << std::endl;
        for( snark::ur::modes_t::const_iterator it = snark::ur::modes.begin(); it != snark::ur::modes.end(); ++it ) { std::cerr << "    " << it->left << ": " << it->right << std::endl; }
        std::cerr << std::endl;
        std::cerr << "joint modes: " << std::endl;
        for( snark::ur::modes_t::const_iterator it = snark::ur::joint_modes.begin(); it != snark::ur::joint_modes.end(); ++it ) { std::cerr << "    " << it->left << ": " << it->right << std::endl; }
        std::cerr << std::endl;
    }
    exit ( -1 );
}

struct status_t 
{
    boost::posix_time::ptime t;
    snark::ur::arm_t arm;
    snark::ur::tool_t tool;
    int mode;
    double time_since_boot;
    
    void set( const snark::ur::packet_t& packet )
    {
        t = boost::posix_time::microsec_clock::universal_time();
        packet.export_to( arm );
        packet.export_to( tool );        
        mode = static_cast< int >( packet.mode() );
        time_since_boot = packet.time_since_boot();
    }
};

namespace comma { namespace visiting {

template <> struct traits< status_t >
{
    template< typename K, typename V > static void visit( const K& k, const status_t& t, V& v )
    {
        v.apply( "t", t.t );
        v.apply( "arm", t.arm );
        v.apply( "tool", t.tool );
        v.apply( "mode",  t.mode );
        v.apply( "time_since_boot", t.time_since_boot );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< status_t >(), ',' ) << std::endl; return 0; }
        comma::csv::options csv;
        csv.full_xpath = true;
        csv.fields = options.value< std::string >( "--fields", "" );
        if( options.exists( "--format" ) ) { std::cout << comma::csv::format::value< status_t >( csv.fields, true ) << std::endl; return 0; }
        if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< status_t >( csv.fields, true ) ); }
        if( options.exists( "--packet-size" ) ) { std::cout << snark::ur::packet_t::size << std::endl; return 0; }
        if( options.exists( "--mode-from-name" ) ) { std::cout << snark::ur::mode_from_name( options.value< std::string >( "--mode-from-name" ) ) << std::endl; return 0; }
        if( options.exists( "--joint-mode-from-name" ) ) { std::cout << snark::ur::joint_mode_from_name( options.value< std::string >( "--joint-mode-from-name" ) ) << std::endl; return 0; }
        if( options.exists( "--mode-to-name" ) ) { std::cout << snark::ur::mode_to_name( options.value< int >( "--mode-to-name" ) ) << std::endl; return 0; }
        if( options.exists( "--joint-mode-to-name" ) ) { std::cout << snark::ur::joint_mode_to_name( options.value< int >( "--joint-mode-to-name" ) ) << std::endl; return 0; }        
        static comma::csv::output_stream< status_t > ostream( std::cout, csv );
        snark::ur::packet_t packet;
        status_t status;
        comma::signal_flag is_shutdown;    
        while( !is_shutdown && std::cin.good() && !std::cin.eof() )
        {
            std::cin.read( packet.data(), snark::ur::packet_t::size );
            if( std::cin.gcount() == 0 ) { break; }
            if( std::cin.gcount() < snark::ur::packet_t::size ) { std::cerr << name() << ": received a packet of length " << std::cin.gcount() << ", expected " << snark::ur::packet_t::size << std::endl; return 1; }
            if( packet.length() != snark::ur::packet_t::size ) { std::cerr << name() << ": received a packet that reports length " << packet.length() << ", expected " << snark::ur::packet_t::size << std::endl; return 1; }
            status.set( packet );
            ostream.write( status );
        }
    }
    catch( std::exception& ex ) { std::cerr << name() << ": " << ex.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception" << std::endl; return 1; }    
}
