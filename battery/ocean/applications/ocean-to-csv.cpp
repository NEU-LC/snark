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
#include <comma/csv/traits.h>
#include <comma/base/types.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include <comma/io/publisher.h>
#include "../units.h"
#include "../commands.h"
#include "../battery.h"
#include "../io_query.h"
#include "../serial_io.h"
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
    std::cerr << "  The Ocean controller must be in Hexadecimal mode (X)." << std::endl;
    std::cerr << "  e.g. socat -u /dev/ttyO1,b19200,raw - | ocean-to-csv --controller-id 1 --beat 0.5 [--status-json|--binary]" << std::endl;
    std::cerr << "  The Ocean controller must be in Message mode (M):" << std::endl;
    std::cerr << "  e.g. ocean-to-csv --query-mode --serial '/dev/ttyO1,19200' --controller-id 1 --beat 0.5 [--status-json|--binary] --publish 'tcp:11002'\"" << std::endl;
    //std::cerr << "  e.g. socat /dev/ttyO1,b19200,raw EXEC:\"ocean-to-csv --query-mode --controller-id 1 --beat 0.5 [--status-json|--binary] --publish 'tcp:11002'\"" << std::endl;
    std::cerr << name() << " options:" << std::endl;
    std::cerr << "    --status-json           - Data output in JSON format with End of Text delimiter char." << std::endl;
    std::cerr << "    --binary                - Data output in binary, default is ascii CSV." << std::endl;
    std::cerr << "    --query-mode            - Alternative mode, query controller for data in message mode. " << std::endl;
    std::cerr << "       The Ocean controller must be in Message mode (M)." << std::endl;
    std::cerr << "    --serial '<device>:<baud rate>'" << std::endl;
    std::cerr << "                            - Optional, for --query-mode e.g. --serial '/dev/ttyO1:19200'." << std::endl;
    std::cerr << "    --publish=              - Do not write to stdout (or stderr for --query-mode) but publish to file|pipe|tcp:<port>." << std::endl;
    std::cerr << "    --verbose               - Output each line received from Ocean Controller, (X) hexadecimal mode only." << std::endl;
    std::cerr << "*   --beat=|-C=             - Minium second/s between status update - 1Hz/beat, floating point e.g. 0.5." << std::endl;
    std::cerr << "*   --controller-id=|-C=    - Controller's ID: 1-9." << std::endl;
    std::cerr << "    --update-all-iteration=<num>|-I=<num>" << std::endl;
    std::cerr << "                            - The <num> th iteration will be query all data - slower than just querying current values. Use with --query-mode only." << std::endl;
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

/// update the controller with recieved data, returns the battery ID of updated battery
/// For(X) Hexadecimal mode
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
static bool is_compact_json = false;

/// Write status to oss publisher
void publish_status( const stats_t& stats, publisher& oss, const comma::csv::options& csv )
{
     if( status_in_json || is_compact_json )
     {
         boost::property_tree::ptree t;
         comma::to_ptree to_ptree( t );
         comma::visiting::apply( to_ptree ).to( stats );
         std::ostringstream ss;
         boost::property_tree::write_json( ss, t, false ); // compact json only   
         oss << ss.str(); 
     }
     else if( is_binary )
     {
         static comma::csv::binary< stats_t > binary( csv );
         static std::vector<char> line( binary.format().size() );
         binary.put( stats, line.data() );
         oss.write( line.data(), line.size());
     }
     else
     {
         static std::string out;
         static comma::csv::ascii< stats_t > ascii( csv );
         ascii.put( stats, out );
         oss << out << '\n';
     }
}
/// Write status to 'oss' stream
void output( const stats_t& stats, const comma::csv::options& csv, std::ostream& oss=std::cout )
{
     if( status_in_json || is_compact_json )
     {
         boost::property_tree::ptree t;
         comma::to_ptree to_ptree( t );
         comma::visiting::apply( to_ptree ).to( stats );
         boost::property_tree::write_json( oss, t, !is_compact_json );    
         // *stream << char(4) << char(3); // End of Transmition

     }
     else
     {
         static comma::csv::output_stream< stats_t > ss( oss, csv, stats );
         ss.write( stats );
     }
     oss.flush();
}

int main( int ac, char** av )
{
    
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
        // Sets up output data options
        comma::csv::options csv;
        csv.fields = options.value< std::string >( "--fields", "" );
        csv.full_xpath = true;
        if( options.exists( "--binary,-b" ) ) csv.format( comma::csv::format::value< stats_t >( csv.fields, false ) );
        
        is_binary = options.exists( "--binary,-b" );
        status_in_json = options.exists( "--status-json" );
        is_compact_json = options.exists( "--compact-json" );
        int controller_id =  options.value< int >( "--controller-id,-C" );
        float beat = options.value< float >( "--beat,-B", 0.4 );
        bool is_query_mode = options.exists( "--query-mode" );
        bool has_publish_stream = options.exists( "--publish" );
        bool verbose = options.exists( "--verbose" );
        boost::optional< std::string > serial_conn; 
        if( options.exists( "--serial" ) ) { serial_conn = options.value< std::string  >( "--serial" ); }
	
        boost::scoped_ptr< publisher > publish;
        if( has_publish_stream )
        {
            publish.reset( new publisher( options.value< std::string >( "--publish" ), 
					  is_binary ? comma::io::mode::binary : comma::io::mode::ascii ) );
        }

        int num_of_batteries = options.value< int >( "--num-of-batteries", 8 );

        if( !is_query_mode ) { std::cerr << name() << ": controller id is " << controller_id << std::endl; }
        
        /// For testing of externl pipe line, produce output in th correct format
        if( options.exists( "--sample" ) )
        {
            stats_t stats;
            stats.time = microsec_clock::universal_time();
            stats.controller.consolidate( num_of_batteries );
            while(1)
            {
                stats.time = microsec_clock::universal_time();
                output( stats, csv );
                sleep( beat );
            }
            
            return 0;
        }
        std::string line;

        // Set it to report status immediately if no command is received
        stats_t stats( controller_id );

        boost::scoped_ptr< snark::ocean::stdio_query_wrapper > io;
        static const boost::posix_time::seconds timeout( 1 );
        
        snark::ocean::serial_query uart;
        if( is_query_mode )
        {
            /// If no serial connection, the program is bound by socat with stdin and stdout
            /// e.g. socat /dev/ttyO1,b19200,raw EXEC:"ocean-to-csv --query-mode -C 1 -B 1" TODO : out of sync problem
            if( serial_conn )
            {
                std::vector< std::string > v = comma::split( *serial_conn, ':' );
                if( v.size() < 2 || v[0].empty() || v[1].empty() ) { std::cerr << name() << ": --serial <device>:<baud needed> needed." << std::endl; usage(1); }
                comma::uint32 baud = 0;
                try{ baud = boost::lexical_cast< comma::uint32 >( v[1] ); } catch( std::exception& ee ) {
                    COMMA_THROW( comma::exception, "baud rate must be a positive number, got: " + v[1] + " : " + ee.what() ); 
                }
                uart.open( v[0],  baud );
                uart.serial.set_timeout( timeout );
            }
            else
            {
                io.reset( new snark::ocean::stdio_query_wrapper() );
                io->set_timeout( timeout );
            }
            
            // try updating one battery, and get the controller in sync with the application
            // TODO: Find out why
            try 
            { 
                std::cout.flush();
                comma::io::select select;
                select.read().add( comma::io::stdin_fd );
                select.wait( 1 );
                while( select.read().ready( comma::io::stdin_fd ) && std::cin.good() )
                { 
                    std::getline( std::cin, line, char(4) );
                    select.wait( 1 );
                }
                if( serial_conn )
                    snark::ocean::query( stats.controller, uart, false /* update_all */ , 1 /*num_of_batteries*/ ); 
                else
                    snark::ocean::query( stats.controller, *io, false /* update_all */ , 1 /*num_of_batteries*/ ); 
            }
            catch( std::exception& e ) {
                std::cerr << name() << " initiating error." << std::endl;
            }
        }

        
        comma::uint16 modulo = options.value< comma::uint16 >( "--update-all-iteration", 10 ); 
        comma::uint16 counter = modulo; // For update all on first iteration
        boost::posix_time::milliseconds beat_timeout( beat* 1000u );
        comma::io::select select;
        select.read().add( comma::io::stdin_fd );
        while( std::cin.good() || serial_conn ) // If using serial connection then std::cin is not read
        {
            if( is_query_mode )
            {
                bool update_all = ( (counter % modulo) == 0 );
                /// query the controller for battery 1 to num_of_batteries
                if( serial_conn ) { 
                    snark::ocean::query( stats.controller, uart, update_all, num_of_batteries );
                }
                else {
                    snark::ocean::query( stats.controller, *io, update_all, num_of_batteries );
                }
                
                /// Data alignment check/ sanity check
                if( modulo <= modulo  ) // do a check once only 
                {
                    double pc = stats.controller.batteries.front().charge_pc;
                    static const comma::int32 pc_limit = 100; // percent
                    if( pc < 0 || pc > pc_limit ) {
                        COMMA_THROW( comma::exception, "data alignment check failed, battery 0's charge % is greater than " 
                            << pc_limit << "% or lower than 0%, it is at " << pc << '%' );
                    }
                }
                
                stats.time = microsec_clock::universal_time();
                stats.controller.consolidate( num_of_batteries );

                if( !has_publish_stream ) { output( stats, csv, std::cerr ); }
                else { publish_status( stats, *publish, csv ); }

                ++counter;
                
                usleep( beat * 1000000u );
                
                continue;
            }
            else
            {
                select.wait( beat_timeout );
                
                if( select.read().ready( comma::io::stdin_fd ) )
                {
                    std::getline( std::cin, line );
                    if( line.empty() ) { continue; }
    
                    snark::ocean::battery_t::strip( line );
                    if( verbose ) { std::cerr << name() << ": " << line <<std::endl; }
    
                    if( line[1] == controller_b::battery_data_char ) // TODO: parse $C line???
                    {
                        // get the battery ID of the data just updated
                        update_controller( stats.controller, line );
                        stats.controller.consolidate( num_of_batteries );
                    }
                }
    
                stats.time = microsec_clock::universal_time();
                if( !has_publish_stream ) { output( stats, csv ); }
                else { publish_status( stats, *publish, csv ); }
            }
        }

        
    }
    catch( comma::exception& ce ) { std::cerr << name() << ": exception thrown: " << ce.what() << std::endl; return 1; }
    catch( std::exception& e ) { std::cerr << name() << ": unknown exception caught: " << e.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception." << std::endl; return 1; }
    
}
