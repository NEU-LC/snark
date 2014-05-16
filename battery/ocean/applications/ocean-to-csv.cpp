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
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/base/types.h>
#include <comma/visiting/apply.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include <comma/csv/stream.h>
#include <boost/property_tree/json_parser.hpp>
#include "../units.h"
#include "../commands.h"
#include "../battery.h"
#include "../io_query.h"
#include "../traits.h"
#include <boost/property_tree/json_parser.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>

static const char* name() {
    return "ocean-to-csv";
}

namespace impl_ {

template < typename T >
std::string str(T t) { return boost::lexical_cast< std::string > ( t ); }
    
}

using snark::ocean::hex_data_t;
using snark::ocean::controller;

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
    std::cerr << "Reads commands from standard input, process and returns a reply to standard output" << std::endl;
    std::cerr << name() << " options:" << std::endl;
    std::cerr << "    --status-json           - Data output in JSON format with End of Text delimiter char." << std::endl;
    std::cerr << "    --binary                - Data output in binary, default is ascii CSV." << std::endl;
    std::cerr << "*   --beat=|-C=             - Minium second/s between status update, floating point e.g. 0.5." << std::endl;
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
            return data.battery_id;
        }
        case 13u:
        {
            hex_data_t< 6 > data;
            ascii< hex_data_t< 6 > >().get( data, v );
            controller & data;
            return data.battery_id;
        }
        case 11u:
        {
            hex_data_t< 5 > data;
            ascii< hex_data_t< 5 > >().get( data, v );
            controller & data;
            return data.battery_id;
        }
        case 9u:
        {
            hex_data_t< 4 > data;
            ascii< hex_data_t< 4 > >().get( data, v );
            controller & data;
            return data.battery_id;
        }
        case 7u:
        {
            hex_data_t< 3 > data;
            ascii< hex_data_t< 3 > >().get( data, v );
            controller & data;
            return data.battery_id;
        }
        case 5u:
        {
            hex_data_t< 2 > data;
            ascii< hex_data_t< 2 > >().get( data, v );
            controller & data;
            return data.battery_id;
        }
        case 3u:
        {
            hex_data_t< 1 > data;
            ascii< hex_data_t< 1 > >().get( data, v );
            controller & data;
            return data.battery_id;
        }
        default:
        {
            COMMA_THROW( comma::exception, "unknown number of hex value pairs found" );
        }
    }

}

static bool is_binary = false;
static bool status_in_json = false;

void output( const stats_t& stats )
{
     if( is_binary )
     {
         static comma::csv::binary< stats_t > binary("","",true, stats );
         static std::vector<char> line( binary.format().size() );
         binary.put( stats, line.data() );
         std::cout.write( line.data(), line.size());
     }
     else if( status_in_json )
     {
         boost::property_tree::ptree t;
         comma::to_ptree to_ptree( t );
         comma::visiting::apply( to_ptree ).to( stats );
         boost::property_tree::write_json( std::cout, t );    
         // *stream << char(4) << char(3); // End of Transmition

     }
     else
     {
         comma::csv::output_stream< stats_t > ss( std::cout );
         ss.write( stats );
     }
     std::cout.flush();
}

int main( int ac, char** av )
{
#ifdef SERVO_VERBOSE
    std::cerr << name() << " Application started:" << std::endl;
#endif
    
    comma::command_line_options options( ac, av );
    if( options.exists( "-h,--help" ) ) { usage( 0 ); }
    
    if( options.exists( "-b,--binary" ) && options.exists( "--status-json") ) { 
        std::cerr << name() << ": --binary|-b cannot be used with --status-json!" << std::endl;
        usage( 1 ); 
    }

    using boost::posix_time::microsec_clock;
    using boost::posix_time::seconds;
    using boost::posix_time::ptime;


    try
    {
        is_binary = options.exists( "--binary,-b" );
        status_in_json = options.exists( "--status-json" );
        int controller_id =  options.value< int >( "--controller-id,-C" );
        float beat = options.value< float >( "--beat,-B" );
        
        bool is_query_mode = options.exists( "--query-mode" );

        int num_of_batteries = 8;
        if( options.exists( "--num-of-batteries" ) ) { num_of_batteries = options.value< int >( "--num-of-batteries" ); }

        std::cerr << name() << ": controller id is " << controller_id << std::endl;
        
        if( options.exists( "--sample" ) )
        {
            stats_t stats;
            stats.time = microsec_clock::universal_time();
            stats.controller.consolidate( num_of_batteries );
            while(1)
            {
                stats.time = microsec_clock::universal_time();
                output( stats );
                sleep( beat );
            }
            
            return 0;
        }

        std::string line;
        // Set it to report status immediately if no command is received
        ptime future = microsec_clock::universal_time();
        stats_t stats( controller_id );
        while( std::cin.good() )
        {
            if( is_query_mode )
            {
                /// query the controller for all batteries
                snark::ocean::query< snark::ocean::stdio_query >( stats.controller, num_of_batteries );
                
                stats.time = microsec_clock::universal_time();
                stats.controller.consolidate( num_of_batteries );

                output( stats );

                future = microsec_clock::universal_time() + seconds( beat );
                
                continue;
            }
            else
            {
                std::getline( std::cin, line );
                if( line.empty() ) { continue; }
    
                //std::cerr << "a: " << line <<std::endl;
                snark::ocean::battery_t::strip( line );
                std::cerr << name() << "d: " << line <<std::endl;
    
                if( line[0] != controller_b::battery_data_char ) continue; // TODO: parse $C line???
    
                // get the battery ID of the data just updated
                int battery_id = update_controller( stats.controller, line );
                // if battery ID is num_of_batteries it means we have updated all batteries
                if( beat > 0 && battery_id == num_of_batteries && microsec_clock::universal_time() >= future )
                {
                    stats.time = microsec_clock::universal_time();
                    stats.controller.consolidate( num_of_batteries );
    
                    output( stats );
    
                    future = microsec_clock::universal_time() + seconds( beat );
                }
            }
        }

        
    }
    catch( comma::exception& ce ) { std::cerr << name() << ": exception thrown: " << ce.what() << std::endl; return 1; }
    catch( std::exception& e ) { std::cerr << name() << ": unknown exception caught: " << e.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception." << std::endl; return 1; }
    
}
