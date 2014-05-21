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
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/base/types.h>
#include <comma/visiting/apply.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include <comma/io/publisher.h>
#include <comma/csv/stream.h>
#include "../units.h"
#include "../commands.h"
#include "../battery.h"
#include "../io_query.h"
#include "../traits.h"

static const char* name() {
    return "ocean-to-csv";
}

namespace impl_ {

template < typename T >
std::string str(T t) { return boost::lexical_cast< std::string > ( t ); }
    
}

using snark::ocean::hex_data_t;
using snark::ocean::controller;
using comma::io::publisher;

// Always have 8 batteries for controller but some of them may not be present
// Use option --num-of-batteries=<no.> so that the average and total values on the controller is calculated correctly
typedef controller< 8 > controller_b;

struct stats_t
{
    boost::posix_time::ptime time;
    controller_b controller;

    stats_t( int id=1 ) : controller( id ) {}
};

namespace comma { namespace visiting { 

template <> struct traits< stats_t >
{
    template< typename K, typename V > static void visit( const K& k, const stats_t& t, V& v )
    {
        v.apply("time", t.time );
        v.apply("controller", t.controller );
    }
};

} } // namespace comma { namespace visiting { 


void usage(int code=1)
{
    std::cerr << std::endl;
    std::cerr << "Accepts or reads Ocean battery data and output them as a status structure - see below." << std::endl;
    std::cerr << "Usage:" << std::endl;
    std::cerr << "  e.g. socat -u /dev/ttyO1,b19200,raw - | ocean-to-csv --controller-id 1 --beat 0.5 [--status-json|--binary]" << std::endl;
    std::cerr << "  e.g. socat /dev/ttyO1,b19200,raw,flusho=1 EXEC:\"ocean-to-csv --query-mode --controller-id 1 --beat 0.5 [--status-json|--binary] 2>results\"" << std::endl;
    std::cerr << name() << " options:" << std::endl;
    std::cerr << "    --status-json           - Data output in JSON format with End of Text delimiter char." << std::endl;
    std::cerr << "    --binary                - Data output in binary, default is ascii CSV." << std::endl;
    std::cerr << "    --query-mode            - Alternative mode, query controller for data in message mode." << std::endl;
    std::cerr << "    --publish=              - Do not write to stdout or stderr for --query-mode but publish, options: file|pipe|tcp:<port>." << std::endl;
    std::cerr << "*   --beat=|-C=             - Minium second/s between status update - 1Hz/beat, floating point e.g. 0.5." << std::endl;
    std::cerr << "*   --controller-id=|-C=    - Controller's ID: 1-9." << std::endl;
    std::cerr << "    [--num-of-batteries=]   - The number of batteries for this controller" << std::endl;
    std::vector< std::string > names = comma::csv::names< stats_t >();
    std::cerr << "    --fields="  << comma::join( names, ',' ) << " total num of fields: " << names.size() << std::endl;
    comma::csv::binary< stats_t > binary;
    std::cerr << "    --format="  << binary.format().string()
              << "  total binary size is "  << binary.format().size()  << std::endl;
    std::cerr << std::endl;
    exit ( code );
}

template < typename T >
comma::csv::ascii< T >& ascii() { 
    static comma::csv::ascii< T > ascii_;
    return ascii_;
}

template < typename T >
comma::csv::binary< T >& binary() { 
    static comma::csv::binary< T > binary_;
    return binary_;
}

/// update the controller with recieved data, returns the battery ID of the data in line 
template < int B >
int update_controller( controller< B >& controller, const std::string& line )
{
    std::vector< std::string > v = comma::split( line, ',');
    switch( v.size() )
    {
        case 15u: // 7 address & value pairs, 7*2 + 1 = 15
        {
            hex_data_t< 7 > data;
            ascii< hex_data_t< 7 > >().get( data, v );
            controller & data;
            return data.type.battery_id();
        }
        case 13u:
        {
            hex_data_t< 6 > data;
            ascii< hex_data_t< 6 > >().get( data, v );
            controller & data;
            return data.type.battery_id();
        }
        case 11u:
        {
            hex_data_t< 5 > data;
            ascii< hex_data_t< 5 > >().get( data, v );
            controller & data;
            return data.type.battery_id();
        }
        case 9u:
        {
            hex_data_t< 4 > data;
            ascii< hex_data_t< 4 > >().get( data, v );
            controller & data;
            return data.type.battery_id();
        }
        case 7u:
        {
            hex_data_t< 3 > data;
            ascii< hex_data_t< 3 > >().get( data, v );
            controller & data;
            return data.type.battery_id();
        }
        case 5u:
        {
            hex_data_t< 2 > data;
            ascii< hex_data_t< 2 > >().get( data, v );
            controller & data;
            return data.type.battery_id();
        }
        case 3u:
        {
            hex_data_t< 1 > data;
            ascii< hex_data_t< 1 > >().get( data, v );
            controller & data;
            return data.type.battery_id();
        }
        default:
        {
            COMMA_THROW( comma::exception, "unknown number of hex value pairs found" );
        }
    }

}

static bool is_binary = false;
static bool status_in_json = false;

void publish_status( const stats_t& stats, publisher& oss )
{
     if( is_binary )
     {
         static comma::csv::binary< stats_t > binary("","",true, stats );
         static std::vector<char> line( binary.format().size() );
         binary.put( stats, line.data() );
         oss.write( line.data(), line.size());
     }
     else
     {
         static std::string out;
         ascii< stats_t >().put( stats, out );
         oss << out << '\n';
     }
}

void output( const stats_t& stats, std::ostream& oss=std::cout )
{
     if( is_binary )
     {
         static comma::csv::binary< stats_t > binary("","",true, stats );
         static std::vector<char> line( binary.format().size() );
         binary.put( stats, line.data() );
         oss.write( line.data(), line.size());
     }
     else if( status_in_json )
     {
         boost::property_tree::ptree t;
         comma::to_ptree to_ptree( t );
         comma::visiting::apply( to_ptree ).to( stats );
         boost::property_tree::write_json( oss, t );    
         // *stream << char(4) << char(3); // End of Transmition

     }
     else
     {
         static comma::csv::output_stream< stats_t > ss( oss );
         ss.write( stats );
     }
     oss.flush();
}

int main( int ac, char** av )
{
    
    using boost::posix_time::microsec_clock;
    using boost::posix_time::seconds;
    using boost::posix_time::ptime;
    using namespace snark::ocean;


    try
    {
        ocean8 cmd = command_bits< 2, address::rel_state_of_charge >::value;
        std::cerr << "writing: " << int(cmd) << std::endl;
        char buf[2];
        buf[0] = cmd;
        buf[1] = '\n';
        std::cout.write( (char*) &buf, 1u );
        std::cout.flush();
        usleep( 400000 );
        ocean8 lsb = std::cin.get();
        if( !std::cin.good() ) { std::cerr << " got EOF " << std::endl; }
        // std::cin.read( (char*) &lsb, 1u );
        std::cerr << "read lsb: " << int(lsb) << std::endl;
        static const ocean8 z = 0;
        std::cout.write( (char*) &z, 1u );
        std::cout.flush();
        usleep( 400000 );
        ocean8 msb = std::cin.get();
        if( !std::cin.good() ) { std::cerr << " got EOF " << std::endl; }
        std::cerr << "read msb: " << int(msb) << std::endl;

        usleep( 400000 );

        cmd = command_bits< 2, address::temperature >::value;
        std::cerr << "writing: " << int(cmd) << std::endl;
        std::cout.write( (char*) &cmd, 1u );

        lsb = std::cin.get();
        if( !std::cin.good() ) { std::cerr << " got EOF " << std::endl; }
        std::cerr << "read lsb: " << int(lsb) << std::endl;
        std::cout.write( (char*)&z, 1u );
       
        msb = std::cin.get();
        if( !std::cin.good() ) { std::cerr << " got EOF " << std::endl; }
        std::cerr << "read msb: " << int(msb) << std::endl;

        stdio_query io;
        std::cerr << " cmd_query: " << cmd << " val: " << cmd_query< 2 , address::rel_state_of_charge >( io );
        
    }
    catch( comma::exception& ce ) { std::cerr << name() << ": exception thrown: " << ce.what() << std::endl; return 1; }
    catch( std::exception& e ) { std::cerr << name() << ": unknown exception caught: " << e.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception." << std::endl; return 1; }
    
}
