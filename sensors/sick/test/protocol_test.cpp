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
#include <boost/asio/ip/tcp.hpp>
#include <comma/application/signal_flag.h>
#include <snark/sensors/sick/protocol.h>
#include <gtest/gtest.h>

TEST( sick, protocol )
{
    boost::asio::ip::tcp::endpoint e( boost::asio::ip::address::from_string( "127.0.0.1" ), 1234 );

    boost::asio::ip::tcp::iostream stream( e );
    if( !stream ) { std::cerr << "---> connect failed" << std::endl; exit( -1 ); }
    std::cerr << "---> connected" << std::endl;
    if( !stream.good() || stream.eof() ) { std::cerr << "---> stream bad" << std::endl; exit( -1 ); }
    
    stream << "hello world" << std::endl;
    stream.flush();
    std::cerr << "---> hello" << std::endl;
    
    stream.write( "goodbye moon\n", ::strlen( "goodbye moon\n" ) );
    std::cerr << "---> bye" << std::endl;
    
    comma::signal_flag flag;
    
    std::string line( 1024, 0 );
    
    std::cerr << "---> reading..." << std::endl;
    stream.read( &line[0], 6 );
    std::cerr << "---> signal flag is " << ( flag ? "" : "not " ) << "set" << std::endl;
    
    std::cerr << "---> got " << stream.gcount() << " byte(s): " << line << std::endl;    
}

int main( int argc, char* argv[] )
{
    ::testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
    return 0;
}
