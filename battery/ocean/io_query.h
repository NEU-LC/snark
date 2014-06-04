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

#ifndef SNARK_OCEAN_IO_QUERY_H
#define SNARK_OCEAN_IO_QUERY_H
#include <comma/base/types.h>
#include <comma/io/select.h>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/array.hpp>
#include <boost/utility/binary.hpp>
#include <vector>
#include <iomanip>
#include <iostream>
#include "commands.h"
#include "battery.h"
#include "io_interface.h"
#include "serial_io.h"
#include "stdio_query.h"

namespace snark { namespace ocean {
    
class stdio_query_wrapper : public query_io< stdio_query_wrapper > 
{
public:
    stdio_query_wrapper() : timeout_( boost::posix_time::milliseconds( 1000 ) )  {}

    void set_timeout_impl( const boost::posix_time::time_duration& duration ) { timeout_ = duration; }

    ocean8 read_impl()
    {
        std::string data( 1u, '\0' );
        io_.receive( data, timeout_ ); // throws on timeout( cancel error code ) or failure

        return data[0];
    }
    
    void write_impl( ocean8 value )
    {
        io_.write( (const char*) &value, 1u );
        std::cout.flush();
    };

    boost::posix_time::time_duration timeout_;
    stdio_query io_;
};

struct serial_query : public query_io< serial_query >
{
    serial_query() {}
    serial_query( const std::string& device, comma::uint32 baud ) : serial( device, baud ) {}
    void set_timeout_impl( boost::posix_time::time_duration duration ) { serial.set_timeout( duration ); }

    void open( const std::string& device, comma::uint32 baud ){ serial.open( device, baud );}
    
    ocean8 read_impl() 
    {
        ocean8 c; 
        serial.read( (char*) &c, 1u );
        // std::cerr << "read: " << int(c) << std::endl;
        return c; 
    }
    
    void write_impl( ocean8 value )
    {
        // std::cerr << "write: " << int(value) << std::endl;
        serial.write( (const char*) &value, 1u ); 
    }
    
    serial_io serial;
};

template < int B, ocean8 ADDR, typename IO >
comma::uint16 cmd_query( IO& io )
{
    io.write( command_bits< ocean8( B ), ADDR >::value );
    // std::cerr << "Readding..." << std::endl;
    comma::uint16 lsbyte ( io.read() );
    io.write( ocean8(0) );
    comma::uint16 msbyte ( io.read() );
    
    // return comma::uint16( ( msbyte << 8 )  | lsbyte );
    comma::uint16 result = ( ( msbyte << 8 )  | lsbyte );
 
    // std::cerr << "query " << B << " address: " << int(ADDR) 
    //           << " command: " <<  int( command_bits< ocean8( B ), ADDR >::value )
    //           << " lsb: " << int( lsbyte ) << " msb: " << int( msbyte )
    //           << " value: " << result << std::endl;
    return result;
}

template < int B, ocean8 ADDR, typename IO >
data_t query( IO & io )
{
    return data_t( ADDR, cmd_query< B, ADDR >( io ) );
}

namespace impl_ {

/// Looping through each battery for N to 1, B=N initially
/// N is total number of battery, B is battery number
template < int B, typename CONTROLLER, typename IO >
struct update_controller
{
    static void update( CONTROLLER& controller, IO& io, bool update_all=true )
    {
        controller.batteries[B-1] & query< B, address::current >( io );
        if( update_all )
        {
            controller.batteries[B-1] & query< B, address::average_current >( io );
            controller.batteries[B-1] & query< B, address::temperature >( io );
            controller.batteries[B-1] & query< B, address::voltage >( io );
            controller.batteries[B-1] & query< B, address::rel_state_of_charge >( io );
            controller.batteries[B-1] & query< B, address::remaining_capacity >( io );
            controller.batteries[B-1] & query< B, address::run_time_to_empty >( io );
            controller.batteries[B-1] & query< B, address::status >( io );
        }
        
        update_controller< B - 1, CONTROLLER, IO >::update( controller, io, update_all );
    }
};

template < typename CONTROLLER, typename IO >
struct update_controller< 0, CONTROLLER, IO >
{
    static void update( CONTROLLER& controller, IO& io, bool update_all=true ) {}
};
    
} //namespace impl_ {

    
///
/// Query the HW controller via IO type and populate the controller< N > with data for up to a specified number of batteries.
///  num_of_batteries is the number of batteries to query e.g. 4 for querying batteries 1-4
template < typename IO, int N >
void query( controller< N >& controller, IO& io, bool update_all=true, char num_of_batteries=N )
{
    typedef  ocean::controller< N > controller_t;
    switch( num_of_batteries )
    {
        case 8:  { impl_::update_controller< 8, controller_t, IO >::update( controller, io, update_all ); break; }
        case 7:  { impl_::update_controller< 7, controller_t, IO >::update( controller, io, update_all ); break; }
        case 6:  { impl_::update_controller< 6, controller_t, IO >::update( controller, io, update_all ); break; }
        case 5:  { impl_::update_controller< 5, controller_t, IO >::update( controller, io, update_all ); break; }
        case 4:  { impl_::update_controller< 4, controller_t, IO >::update( controller, io, update_all ); break; }
        case 3:  { impl_::update_controller< 3, controller_t, IO >::update( controller, io, update_all ); break; }
        case 2:  { impl_::update_controller< 2, controller_t, IO >::update( controller, io, update_all ); break; }
        default: { impl_::update_controller< 1, controller_t, IO >::update( controller, io, update_all ); break; }
    }
}
    
} } // namespace snark { namespace ocean {

#endif // SNARK_OCEAN_IO_QUERY_H
