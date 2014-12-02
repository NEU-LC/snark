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

#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/string/split.h>
#include <comma/string/string.h>
#include <comma/application/signal_flag.h>
#include "packet.h"
#include "arm.h"

static const char* name() { return "ur-arm-status"; }

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "take arm status feed on stdin, output csv data to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h: show this message" << std::endl;
    std::cerr << "    --binary,-b: output binary equivalent of csv" << std::endl;
    std::cerr << "    --format: output binary format for given fields to stdout and exit" << std::endl;
    std::cerr << "    --output-fields: output field names and exit" << std::endl;
    std::cerr << "    --packet-size: output packet-size and exit" << std::endl;
    std::cerr << "    --robot-mode-from-name=<name>: output the integer corresponding to the given robot mode name and exit" << std::endl;
    std::cerr << "    --joint-mode-from-name=<name>: same as above for joint modes" << std::endl;
    std::cerr << "    --robot-mode-to-name=<mode>: output the name of the given robot mode integer and exit" << std::endl;
    std::cerr << "    --joint-mode-to-name=<mode>: same as above for joint modes" << std::endl;    
    std::cerr << std::endl;
    std::cerr << "examples: " << std::endl;
    std::cerr << "    socat -u tcp:robot.arm:30003 - | " << name() << " --fields=t,mode,joints/mode" << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "robot modes: " << std::endl;
        for( comma::ur::modes_t::const_iterator it = comma::ur::robot_modes.begin(); it != comma::ur::robot_modes.end(); ++it ) { std::cerr << "    " << it->left << ": " << it->right << std::endl; }
        std::cerr << std::endl;
        std::cerr << "joint modes: " << std::endl;
        for( comma::ur::modes_t::const_iterator it = comma::ur::joint_modes.begin(); it != comma::ur::joint_modes.end(); ++it ) { std::cerr << "    " << it->left << ": " << it->right << std::endl; }
        std::cerr << std::endl;
    }
    exit ( -1 );
}

struct joints_t 
{
    boost::array< double, comma::ur::number_of_joints > angle;
    boost::array< double, comma::ur::number_of_joints > velocity;
    boost::array< double, comma::ur::number_of_joints > current;
    boost::array< double, comma::ur::number_of_joints > temperature;
    typedef comma::int32 mode_t;
    boost::array< mode_t, comma::ur::number_of_joints > mode;
};

struct tool_t
{
    boost::array< double, comma::ur::number_of_pose_fields > pose;
    boost::array< double, comma::ur::number_of_pose_fields > speed;
    boost::array< double, comma::ur::number_of_pose_fields > force;
};

struct status_t 
{
    boost::posix_time::ptime t;
    joints_t joints;
    tool_t tool;
    typedef comma::int32 mode_t;
    mode_t mode;
    double time_since_boot;
};

namespace comma { namespace visiting {

template < > struct traits< joints_t >
{
    template< typename K, typename V > static void visit( const K& k, const joints_t& t, V& v )
    {
        v.apply( "angle", t.angle );
        v.apply( "velocity", t.velocity );
        v.apply( "current", t.current );
        v.apply( "temperature", t.temperature );
        v.apply( "mode", t.mode );
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
        v.apply( "joints", t.joints );
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
        //if( options.exists( "-h,--help" ) ) { usage(); }
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< status_t >(), ',' ) << std::endl; return 0; }
        comma::csv::options csv;
        csv.full_xpath = true;
        csv.fields = options.value< std::string >( "--fields", "" );
        if( options.exists( "--format" ) ) { std::cout << comma::csv::format::value< status_t >( csv.fields, true ) << std::endl; return 0; }
        if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< status_t >( csv.fields, true ) ); }
        if( options.exists( "--packet-size" ) ) { std::cout << comma::ur::packet_t::size << std::endl; return 0; }
        if( options.exists( "--robot-mode-from-name" ) ) { std::cout << comma::ur::robot_mode_from_name( options.value< std::string >( "--robot-mode-from-name" ) ) << std::endl; return 0; }
        if( options.exists( "--joint-mode-from-name" ) ) { std::cout << comma::ur::joint_mode_from_name( options.value< std::string >( "--joint-mode-from-name" ) ) << std::endl; return 0; }
        if( options.exists( "--robot-mode-to-name" ) ) { std::cout << comma::ur::robot_mode_to_name( options.value< int >( "--robot-mode-to-name" ) ) << std::endl; return 0; }
        if( options.exists( "--joint-mode-to-name" ) ) { std::cout << comma::ur::joint_mode_to_name( options.value< int >( "--joint-mode-to-name" ) ) << std::endl; return 0; }        
        static comma::csv::output_stream< status_t > ostream( std::cout, csv );
        comma::ur::packet_t packet;
        status_t status;
        comma::signal_flag is_shutdown;    
        while( !is_shutdown && std::cin.good() && !std::cin.eof() )
        {
            std::cin.read( packet.data(), comma::ur::packet_t::size );
            if( std::cin.gcount() == 0 ) { break; }
            if( std::cin.gcount() < comma::ur::packet_t::size ) { std::cerr << name() << ": received a packet of length " << std::cin.gcount() << ", expected " << comma::ur::packet_t::size << std::endl; return 1; }
            if( packet.length() != comma::ur::packet_t::size ) { std::cerr << name() << ": received a packet that reports length " << packet.length() << ", expected " << comma::ur::packet_t::size << std::endl; return 1; }
            status.t = boost::posix_time::microsec_clock::universal_time();
            status.mode = static_cast< status_t::mode_t >( packet.robot_mode() );
            status.time_since_boot = packet.time_since_boot();    
            for( unsigned int i = 0; i < comma::ur::number_of_joints; ++i )
            { 
                status.joints.angle[i] = packet.actual_joint_positions[i]();
                status.joints.velocity[i] = packet.actual_joint_velocities[i]();
                status.joints.current[i] = packet.actual_joint_currents[i]();
                status.joints.temperature[i] = packet.joint_temperatures[i]();
                status.joints.mode[i] = static_cast< joints_t::mode_t >( packet.joint_modes[i]() );
            }
            for( unsigned int i = 0; i < comma::ur::number_of_pose_fields; ++i )
            {
                status.tool.pose[i] = packet.tool_pose[i]();
                status.tool.speed[i] = packet.tool_speed[i]();
                status.tool.force[i] = packet.tool_generalised_forces[i]();
            }
            ostream.write( status );
        }
    }
    catch( std::exception& ex ) { std::cerr << name() << ": " << ex.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception" << std::endl; return 1; }    
}
