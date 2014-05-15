#ifndef SNARK_OCEAN_IO_QUERY_H
#define SNARK_OCEAN_IO_QUERY_H
#include <comma/base/types.h>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/array.hpp>
#include <boost/utility/binary.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <vector>
#include <iomanip>
#include <iostream>
#include "commands.h"
#include "battery.h"
#include <aero/navigation/applications/wmm/GeomagnetismHeader.h>

namespace snark { namespace ocean {
    
class stdio_query 
{
public:
    ocean8 read()
    {
        char c;
        std::cin.read( &c, 1u );
        return c;
    }
    
    void write( ocean8 value )
    {
        std::cout.write( (const char*) &value, 1u );
    };
};

template < int B, ocean8 ADDR, typename IO >
comma::uint16 cmd_query( )
{
    IO io;
    io.write( command_bits< ocean8( B ), ADDR >::value );
    ocean8 lsbyte = io.read();
    io.write( ocean8(0) );
    comma::uint16 msbyte = io.read();
    
    return comma::uint16( ( msbyte << 8 )  | lsbyte );
//     comma::uint16 result = ( ( msbyte << 8 )  | lsbyte );
    
//     std::cerr << "query " << B << " address: " << int(ADDR) 
//               << " lsb: " << int( lsbyte ) << " msb: " << int( msbyte )
//               << " value: " << result << std::endl;
//     return result;
}

template < int B, ocean8 ADDR, typename IO >
data_t query()
{
    return data_t( ADDR, cmd_query< B, ADDR, IO >() );
}

namespace impl_ {

/// Looping through each battery for N to 1, B=N initially
/// N is total number of battery, B is battery number
template < int B, int N, typename IO >
struct update_controller
{
    static void update( controller< N >& controller )
    {
        controller.batteries[B-1] & query< B, address::current, IO >();
        controller.batteries[B-1] & query< B, address::average_current, IO >();
        controller.batteries[B-1] & query< B, address::temperature, IO >();
        controller.batteries[B-1] & query< B, address::voltage, IO >();
        controller.batteries[B-1] & query< B, address::rel_state_of_charge, IO >();
        controller.batteries[B-1] & query< B, address::remaining_capacity, IO >();
        controller.batteries[B-1] & query< B, address::run_time_to_empty, IO >();
        controller.batteries[B-1] & query< B, address::status, IO >();
        
        update_controller< B - 1, N, IO >::update( controller );
    }
};

template < int N, typename IO >
struct update_controller< 0, N, IO >
{
    static void update( controller< N >& controller ) {}
};
    
} //namespace impl_ {

    
template < int N, typename IO >
bool query( controller< N >& controller )
{
    impl_::update_controller< N, N, IO >::update( controller );
    return true;
}
    
} } // namespace snark { namespace ocean {

#endif // SNARK_OCEAN_IO_QUERY_H