// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

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
