
#ifndef SNARK_OCEAN_COMMANDS_H
#define SNARK_OCEAN_COMMANDS_H
#include <comma/base/types.h>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/array.hpp>
#include <boost/utility/binary.hpp>
#include <vector>
#include <iomanip>
#include <iostream>
#include "hex_value.h"

namespace snark { namespace ocean {

typedef unsigned char ocean8;    

/// Gives you an ocean command where the bits 7-5 is battery number, 4-0 is the register address 
template < ocean8 B, ocean8 ADDR > struct command_bits {
    static const ocean8 address_mask = BOOST_BINARY( 11111 );
    static const ocean8 value = ( B << 5 ) | ( ADDR & address_mask );    
};


} } // namespace snark { namespace ocean {

#endif // SNARK_OCEAN_COMMANDS_H
