#ifndef WIN32
#include <stdlib.h>
#endif

#include <iostream>
#include <snark/timing/time.h>
#include <snark/sensors/sick/packets.h>
#include <gtest/gtest.h>

using namespace snark::sick::ldmrs;

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

