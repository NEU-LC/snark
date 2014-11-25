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
#include <comma/packed/packed.h>
//#include "../data.h"

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
    std::cerr << "    socat -u -T 1 tcp:robot-arm:30003 - | ur-arm-status --fields=t,robot_mode,arm/modes" << std::endl;
    std::cerr << std::endl;
    exit ( -1 );
}

static const unsigned int number_of_joints = 6;
static const unsigned int number_of_tool_fields = 6;
typedef boost::array< comma::packed::net_float64, number_of_joints > net_arm_array_t;
typedef boost::array< comma::packed::net_float64, number_of_tool_fields > net_tool_array_t;
struct packet_t : public comma::packed::packed_struct< packet_t, 812  >
{
    comma::packed::big_endian_uint32 length;
    comma::packed::net_float64 time_since_boot;
    comma::packed::string< 240 > dummy1;
    net_arm_array_t joint_positions; 
    net_arm_array_t joint_velocities;
    net_arm_array_t joint_currents;
    comma::packed::string< 48 > dummy2;
    net_tool_array_t tool_pose;
    net_tool_array_t tool_speed;
    net_tool_array_t tool_force;
    comma::packed::string< 104 > dummy3;
    net_arm_array_t  joint_temperatures;
    comma::packed::string< 16 > dummy4;
    comma::packed::net_float64 robot_mode;
    net_arm_array_t joint_modes;
};

typedef boost::array< double, number_of_joints > arm_array_t;
struct arm_t 
{
    arm_array_t angles;
    arm_array_t velocities;
    arm_array_t currents;
    arm_array_t temperatures;
    arm_array_t modes;
};

typedef boost::array< double, number_of_tool_fields > tool_array_t;
struct tool_t
{
    tool_array_t pose;
    tool_array_t speed;
    tool_array_t force;
};

struct status_t 
{
    boost::posix_time::ptime t;
    arm_t arm;
    tool_t tool;
    comma::uint32 robot_mode;
    comma::uint32 packet_length;
    double time_since_boot;
};

namespace comma { namespace visiting {

template < > struct traits< arm_t >
{
    template< typename K, typename V > static void visit( const K& k, const arm_t& t, V& v )
    {
        v.apply( "angles", t.angles );
        v.apply( "velocities", t.velocities );
        v.apply( "currents", t.currents );
        v.apply( "temperatures", t.temperatures );
        v.apply( "modes", t.modes );
    }
};

template < > struct traits< tool_t >
{
    template< typename K, typename V > static void visit( const K& k, const tool_t& t, V& v )
    {
        v.apply( "pose", t.pose );
        v.apply( "speed", t.speed );
        v.apply( "force", t.force );        
    }
};
    
template < > struct traits< status_t >
{
    template< typename K, typename V > static void visit( const K& k, const status_t& t, V& v )
    {
        v.apply( "t", t.t );
        v.apply( "arm", t.arm );
        v.apply( "tool", t.tool );
        v.apply( "robot_mode",  t.robot_mode );
        v.apply( "packet_length", t.packet_length );
        v.apply( "time_since_boot", t.time_since_boot );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "-h,--help" ) ) { usage(); }
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< status_t >(), ',' ) << std::endl; return 0; }
        comma::csv::options csv;
        csv.full_xpath = true;
        csv.fields = options.value< std::string >( "--fields", "" );
        if( options.exists( "--format" ) ) { std::cout << comma::csv::format::value< status_t >( csv.fields, true ) << std::endl; return 0; }
        if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< status_t >( csv.fields, true ) ); }
        static comma::csv::output_stream< status_t > ostream( std::cout, csv );
        packet_t packet;
        status_t status;
        comma::signal_flag is_shutdown;    
        while( !is_shutdown && std::cin.good() && !std::cin.eof() )
        {
            std::cin.read( packet.data(), packet_t::size );
            if( packet.length() != packet_t::size ) { std::cerr << name() << ": expected packet length " << packet_t::size << ", got " << packet.length() << std::endl; return 1; }
            status.t = boost::posix_time::microsec_clock::local_time();
            status.robot_mode = static_cast< comma::uint32 >( packet.robot_mode() );
            status.packet_length = packet.length();
            status.time_since_boot = packet.time_since_boot();    
            for( unsigned int i = 0; i < number_of_joints; ++i)
            { 
                status.arm.angles[i] = packet.joint_positions[i]();
                status.arm.velocities[i] = packet.joint_velocities[i]();
                status.arm.currents[i] = packet.joint_currents[i]();
                status.arm.temperatures[i] = packet.joint_temperatures[i]();
                status.arm.modes[i] = static_cast< comma::uint32 >( packet.joint_modes[i]() );
            }
            for( unsigned int i = 0; i < number_of_tool_fields; ++i)
            {
                status.tool.pose[i] = packet.tool_pose[i]();
                status.tool.speed[i] = packet.tool_speed[i]();
                status.tool.force[i] = packet.tool_force[i]();
            }
            ostream.write( status );
        }
    }
    catch( std::exception& ex ) { std::cerr << name() << ": " << ex.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception" << std::endl; return 1; }    
}
