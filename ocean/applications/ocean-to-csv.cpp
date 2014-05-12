#include <stdint.h>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/base/types.h>
#include <comma/visiting/apply.h>
#include <comma/name_value/ptree.h>
#include <comma/csv/stream.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include "../units.h"
#include "../battery.h"
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

#ifndef BATTERY_NUM
#define BATTERY_NUM (4)
#endif

using snark::ocean::hex_data_t;
using snark::ocean::controller_t;

typedef controller_t< BATTERY_NUM > controller_b;

void usage(int code=1)
{
    std::cerr << std::endl;
    std::cerr << "Reads commands from standard input, process and returns a reply to standard output" << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --status-json           - Data output in JSON format with End of Text delimiter char." << std::endl;
    std::cerr << "    --binary                - Data output in binary, default is ascii CSV." << std::endl;
    std::cerr << "    --fields="  << comma::join( comma::csv::names< controller_b >(), ',' ) << std::endl;
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

template < int B >
void update_controller( controller_t< B >& controller, const std::string& line )
{
    std::vector< std::string > v = comma::split( line, ',');
    switch( v.size() )
    {
        case 15u: // 7 address & value pairs, 7*2 + 1 = 15
        {
            hex_data_t< 7 > data;
            ascii< hex_data_t< 7 > >().get( data, v );
            controller & data;
            break;
        }
        case 13u:
        {
            hex_data_t< 6 > data;
            ascii< hex_data_t< 6 > >().get( data, v );
            controller & data;
            break;
        }
        case 11u:
        {
            hex_data_t< 5 > data;
            ascii< hex_data_t< 5 > >().get( data, v );
            controller & data;
            break;
        }
        case 9u:
        {
            hex_data_t< 4 > data;
            ascii< hex_data_t< 4 > >().get( data, v );
            controller & data;
            break;
        }
        case 7u:
        {
            hex_data_t< 3 > data;
            ascii< hex_data_t< 3 > >().get( data, v );
            controller & data;
            break;
        }
        case 5u:
        {
            hex_data_t< 2 > data;
            ascii< hex_data_t< 2 > >().get( data, v );
            controller & data;
            break;
        }
        case 3u:
        {
            hex_data_t< 1 > data;
            ascii< hex_data_t< 1 > >().get( data, v );
            controller & data;
            break;
        }
        default:
        {
            COMMA_THROW( comma::exception, "unknown number of hex value pairs found" );
            break;
        }
    }

}

int main( int ac, char** av )
{
#ifdef SERVO_VERBOSE
    std::cerr << "Status daemon started:" << std::endl;
#endif
    
    comma::command_line_options options( ac, av );
    if( options.exists( "-h,--help" ) ) { usage( 0 ); }
    
    if( options.exists( "-b,--binary" ) && options.exists( "--status-json") ) { 
        std::cerr << name() << ": --binary|-b cannot be used with --status-json!" << std::endl;
        usage( 1 ); 
    }



    try
    {
        bool is_binary = options.exists( "--binary,-b" );
        int controller_id =  options.value< int >( "--controller-id,-C" );
        float beat = options.value< float >( "--beat,-B" );

        std::cerr << "controller id: " << controller_id << std::endl;

        std::string line;
        // Set it to report status immediately if no command is received
        using boost::posix_time::microsec_clock;
        using boost::posix_time::seconds;
        using boost::posix_time::ptime;
        ptime future = microsec_clock::universal_time() + seconds( 5 );
        controller_b controller( controller_id );
        while( std::cin.good() )
        {
            std::getline( std::cin, line );
            if( line.empty() ) { continue; }

            // std::cerr << "a: " << line <<std::endl;
            snark::ocean::strip( line );
            std::cerr << "d: " << line <<std::endl;

            if( line[0] != 'B' ) continue; // TODO: parse $C line???

            update_controller( controller, line );

            if( beat > 0 && microsec_clock::universal_time() >= future )
            {
                controller.consolidate();

                if( is_binary )
                {
                    static comma::csv::binary< controller_b > binary("","",true, controller );
                    static std::vector<char> line( binary.format().size() );
                    binary.put( controller, line.data() );
                    std::cout.write( line.data(), line.size());
                }
                else
                {
                    comma::csv::output_stream< controller_b > ss( std::cout );
                    ss.write( controller );
                    std::cout.flush();
                }
                future = microsec_clock::universal_time() + seconds( beat );
            }
        }

        
    }
    catch( comma::exception& ce ) { std::cerr << name() << ": exception thrown: " << ce.what() << std::endl; return 1; }
    catch( std::exception& e ) { std::cerr << name() << ": unknown exception caught: " << e.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception." << std::endl; return 1; }
    
}
