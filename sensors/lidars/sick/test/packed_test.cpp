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
// 3. Neither the name of the University of Sydney nor the
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


#ifndef WIN32
#include <stdlib.h>
#endif

#include <iostream>
#include "../../../../timing/time.h"
#include "../ibeo/packets.h"
#include <gtest/gtest.h>

using namespace snark::sick::ibeo;

template < typename command >
static void testcommand()
{
    command c;
    EXPECT_EQ( c.command_header.id(), static_cast< unsigned int >( command::id ) );
    typename command::response response;
    EXPECT_EQ( response.header.id(), static_cast< unsigned int >( command::id ) );
    EXPECT_TRUE( response.ok() );
    EXPECT_TRUE( response.matches( c.command_header.id() ) );
    response.fail();
    EXPECT_FALSE( response.ok() );
    EXPECT_TRUE( response.matches( c.command_header.id() ) );
}

TEST( packed, reset_dsp )
{
    commands::reset_dsp command;
    EXPECT_EQ( command.command_header.id(), static_cast< unsigned int >( commands::reset_dsp::id ) );
}

TEST( packed, commands )
{
    testcommand< commands::get_status >();
    testcommand< commands::save_configuration >();
    testcommand< commands::set >();
    testcommand< commands::get >();
    testcommand< commands::reset >();
    testcommand< commands::start >();
    testcommand< commands::stop >();
    testcommand< commands::set_ntp_seconds >();
    testcommand< commands::set_ntp_fractions >();
}

TEST( packed, header )
{
    unsigned char buf[] = { 175,254,192,194,0,0,0,0,0,0,31,138,0,0,34,2,0,0,0,145,176,76,235,8,82,6,11,3,0,0,128,99 };
    header* h = reinterpret_cast< header* >( buf );
    std::cerr << "valid     : " << h->valid() << std::endl;
    std::cerr << "type      : 0x" << std::hex << h->type() << std::dec << std::endl;
    std::cerr << "size      : " << h->payload_size() << std::endl;
    std::cerr << "seconds   : " << h->t.seconds() << std::endl;
    std::cerr << "fractions : " << h->t.fractions() << std::endl;
    
    boost::posix_time::ptime e( snark::timing::epoch );
    boost::posix_time::ptime t( e );
    t -= ( boost::posix_time::seconds( 1 ) + boost::posix_time::microsec( 1000 ) );
    std::cerr << "---> total seconds: " << ( t - e ).total_seconds() << std::endl;
    std::cerr << "---> microseconds: " << ( ( t - e ).total_microseconds() - ( t - e ).total_seconds() * 1000000 ) << std::endl;
}

