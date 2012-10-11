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
