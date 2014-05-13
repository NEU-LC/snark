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
    
    std::size_t write( ocean8 value )
    {
        std::cout.write( value );
    };
};

template < int B, ocean8 ADDR, typename IO=stdio_query >
comma::uint16 query( )
{
    IO io;
    io.write( command_bits< ocean8( B ), ADDR >::value );
    ocean8 lsbyte = io.read();
    io.write( ocean8(0) );
    ocean8 msbyte = io.read();
    
    return ( ( msbyte << 8 ) | lsbyte );
}
    
template < int N >
bool query( controller_t< N >& controller )
{
    return true;
}
    
} } // namespace snark { namespace ocean {

#endif // SNARK_OCEAN_IO_QUERY_H