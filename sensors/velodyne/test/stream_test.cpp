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

#ifndef WIN32
#include <stdlib.h>
#endif

#include <iostream>
#include <gtest/gtest.h>
#include <snark/sensors/velodyne/stream.h>

#include <snark/sensors/velodyne/impl/udp_reader.h>

TEST(db, stream)
{
    // todo

    std::cerr << "--> 0" << std::endl;
    snark::udp_reader r1( 12345 );
    std::cerr << "--> 1" << std::endl;
    snark::udp_reader r2( 12345 );
    std::cerr << "--> 2" << std::endl;
    r1.close();
    std::cerr << "--> 3" << std::endl;
    r2.close();
    std::cerr << "--> 4" << std::endl;    
}
