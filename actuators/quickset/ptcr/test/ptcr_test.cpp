#include <gtest/gtest.h>
#include <snark/actuators/quickset/ptcr/commands.h>
#include <snark/actuators/quickset/ptcr/packet.h>
#include <snark/actuators/quickset/ptcr/protocol.h>

using namespace snark::quickset;

TEST( ptcr, Lrc )
{
    // todo
}

TEST( ptcr, packetConstruction )
{
    { ptcr::packet< ptcr::commands::get_status > p; }
    { ptcr::packet< ptcr::commands::get_status::response > p( ptcr::constants::ack ); }
    { ptcr::packet< ptcr::commands::move_to > p; }
    { ptcr::packet< ptcr::commands::move_to::response > p( ptcr::constants::ack ); }
}

int main( int argc, char* argv[] )
{
    ::testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
    return 0;
}
