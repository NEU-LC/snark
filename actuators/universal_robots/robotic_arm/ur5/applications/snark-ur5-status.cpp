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
#include "../../traits.h"
#include "../../data.h"
#include "../transforms/transforms.h"

const char* name() { return "snark-ur10-status: "; }

namespace impl_ {

template < typename T >
std::string str(T t) { return boost::lexical_cast< std::string > ( t ); }
    
} // namespace impl_ {

namespace arm = snark::ur::robotic_arm;

void usage(int code=1)
{
    std::cerr << std::endl;
    std::cerr << name() << std::endl;
    std::cerr << "example: socat -u -T 1 tcp:robot-arm:30003 - | snark-ur5-status [--fields=] [--binary | -b | --json | --compact-json]" << std::endl;
    std::cerr << "         Reads in robotic-arm's real time status information ( network byte order ) and output the status in host byte order in any format." << std::endl;
    std::cerr << "         Approximately 20ms between statuses." << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h:            show this message" << std::endl;
    std::cerr << "    --binary,-b:          output in binary." << std::endl;
    std::cerr << "    --json,-j:            output in json." << std::endl;
    std::cerr << "    --compact-json,-cj:   output in single line json without whitespaces or new lines." << std::endl;
    std::cerr << "    --host-byte-order:    input data is binary in host-byte-order, it assumes network order by default." << std::endl;
    std::cerr << "    --format:             displays the binary format of output data in host byte order." << std::endl;
    std::cerr << "    --flip-laser-pitch:   flip the pitch pos/neg sign for laser-position's orientation." << std::endl;
    typedef arm::status_t status_t;
    comma::csv::binary< status_t > binary;
    std::cerr << "Robot arm's status:" << std::endl;
    std::cerr << "   format: " << binary.format().string() << " total size is " << binary.format().size() << " bytes" << std::endl;
    std::vector< std::string > names = comma::csv::names< status_t >();
    std::cerr << "   fields: " << comma::join( names, ','  ) << " number of fields: " << names.size() << std::endl;
    std::cerr << std::endl;
    exit ( code );
}


template < typename T > 
comma::csv::ascii< T >& ascii( )
{
    static comma::csv::ascii< T > ascii_;
    return ascii_;
}

int main( int ac, char** av )
{
    comma::signal_flag signaled;
    
    comma::command_line_options options( ac, av );
    if( options.exists( "-h,--help" ) ) { usage( 0 ); }
    if( options.exists( "--format" ) ) { std::cout << comma::csv::format::value< arm::status_t >( "", false ) << std::endl; return 0; }
    
    using boost::posix_time::microsec_clock;
    using boost::posix_time::seconds;
    using boost::posix_time::ptime;
    using boost::asio::ip::tcp;

    typedef arm::fixed_status status;
    
    std::cerr << name() << "started" << std::endl;

    
    bool is_json = options.exists( "--json,-j" );
    bool is_single_line_json = options.exists( "--compact-json,-cj" );
    bool is_binary = options.exists( "--binary,-b" );
    bool is_flip_laser_pitch = options.exists( "--flip-laser-pitch" );
    
    comma::csv::options csv;
    csv.fields = options.value< std::string >( "--fields", "" );
    csv.full_xpath = true;
    if( is_binary ) { csv.format( comma::csv::format::value< arm::status_t >( csv.fields, true ) ); }
    
    try
    {
        comma::csv::options csv_in;
        csv_in.fields = options.value< std::string >( "--input-fields", "" );
        csv_in.full_xpath = true;
        csv_in.format( comma::csv::format::value< arm::status_t >( csv_in.fields, true ) );

        if( options.exists( "--output-samples" ) )
        {
            // std::cerr << "output format: \n" << csv.format().string() << std::endl;
            // std::cerr << "output fields: \n" << csv.fields << std::endl;
            std::cerr.flush();
            comma::csv::output_stream< arm::status_t > oss( std::cout, csv );
            arm::status_t st;
            st.robot_mode = arm::robotmode::running;
            for( std::size_t i=0; i<arm::joints_num; ++i ) { st.joint_modes[i] = arm::jointmode::running; }  
            st.position.coordinates = Eigen::Vector3d( 1, 2, 3 );
            st.laser_position.coordinates = Eigen::Vector3d( 3, 2, 1 );
            while( !signaled && std::cout.good() )
            {
                st.timestamp = boost::posix_time::microsec_clock::local_time();
                oss.write( st );
                oss.flush();
                usleep( 0.1 * 1000000u );
            }
            return 0;
        }

        bool is_host_order = options.exists( "--host-byte-order" );
        arm::fixed_status arm_status;
        bool first_loop = true;
        arm::status_t state;

        while( !signaled && std::cin.good() )
        {
            if( !is_host_order ) 
            { 
                std::cin.read( arm_status.data(), status::size ); 
                arm_status.get( state );
                /// As TCP rotation data do not make sense, caculate it using the joint angles
                /// We will also override the TCP translation coordinate
                snark::ur::robotic_arm::ur5::tcp_transform( state.joint_angles, state.position, state.laser_position );
                if( is_flip_laser_pitch ) { state.laser_position.orientation.y() = -( state.laser_position.orientation.y() ); }
            }
            else 
            {
                static comma::csv::input_stream< arm::status_t > istream( std::cin, csv_in );
                const arm::status_t* p = istream.read();
                if( p == NULL ) 
                { 
                    if( !std::cin.good() ) { /* std::cerr << name() << "STDIN error" << std::endl; */ return 1; } // happens a lot, when closed on purpose
                    COMMA_THROW( comma::exception, "p is null" ); 
                }
                state = *p;
            }
            
            state.timestamp = boost::posix_time::microsec_clock::local_time();

            // sanity check on the status
            if( first_loop && (
                state.length != arm::fixed_status::size ||
                state.robot_mode < arm::robotmode::running || 
                state.robot_mode > arm::robotmode::safeguard_stop ) ) {
                std::cerr << name() << "mode: " << state.mode_str() << std::endl;
                std::cerr << name() << "failed sanity check, data is not aligned, exiting now..." << std::endl;
                return 1;
            }
            first_loop = false;
            
            if ( is_json || is_single_line_json )
            {
                boost::property_tree::ptree t;
                comma::to_ptree to_ptree( t );
                comma::visiting::apply( to_ptree ).to( state );
                boost::property_tree::write_json( std::cout, t, !is_single_line_json );    
            }
            else 
            { 
                static comma::csv::output_stream< arm::status_t > oss( std::cout, csv );
                oss.write( state );
            }
        }
        
    }
    catch( comma::exception& ce ) { std::cerr << name() << ": exception thrown: " << ce.what() << std::endl; return 1; }
    catch( std::exception& e ) { std::cerr << name() << ": unknown exception caught: " << e.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception." << std::endl; return 1; }
    
}
