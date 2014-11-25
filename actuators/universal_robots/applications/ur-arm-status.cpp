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
#include <boost/asio.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <comma/io/stream.h>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/string/split.h>
#include <comma/base/types.h>
#include <comma/visiting/apply.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include <comma/io/publisher.h>
#include <comma/csv/stream.h>
#include <comma/csv/binary.h>
#include <comma/string/string.h>
#include <comma/application/signal_flag.h>
#include "../traits.h"
#include "../data.h"

static const char* name() { return "ur-arm-status"; }

void usage()
{
    std::cerr << std::endl;
    std::cerr << "take arm status feed on stdin, output csv data to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h:            show this message" << std::endl;
    std::cerr << "    --binary,-b:          output binary equivalent of csv" << std::endl;
    std::cerr << "    --format:             output binary format for given fields to stdout and exit" << std::endl;
    std::cerr << "    --output-fields:      output field names and exit" << std::endl;
    std::cerr << "examples: " << std::endl;
    std::cerr << "    socat -u -T 1 tcp:robot-arm:30003 - | ur-arm-status --fields=timestamp,robot_mode,joint_modes" << std::endl;
    std::cerr << std::endl;
    exit ( -1 );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "-h,--help" ) ) { usage(); }
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< snark::ur::status_t >(), ',' ) << std::endl; return 0; }
        comma::csv::options csv;
        csv.full_xpath = true;
        csv.fields = options.value< std::string >( "--fields", "" );
        if( options.exists( "--format" ) ) { std::cout << comma::csv::format::value< snark::ur::status_t >( csv.fields, true ) << std::endl; return 0; }
        if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< snark::ur::status_t >( csv.fields, true ) ); }
        static comma::csv::output_stream< snark::ur::status_t > ostream( std::cout, csv );
        snark::ur::packet_t packet;
        snark::ur::status_t status;
        comma::signal_flag is_shutdown;        
        while( !is_shutdown && std::cin.good() && !std::cin.eof() )
        {
            std::cin.read( packet.data(), snark::ur::packet_t::size );
            if( packet.length() != snark::ur::packet_t::size ) { std::cerr << name() << ": expected packet length " << snark::ur::packet_t::size << ", got " << packet.length() << std::endl; return 1; }
            
            
            status.set( boost::posix_time::microsec_clock::local_time(), packet ); 
            ostream.write( status );
        }
    }
    catch( std::exception& ex ) { std::cerr << name() << ": " << ex.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception" << std::endl; return 1; }    
}
