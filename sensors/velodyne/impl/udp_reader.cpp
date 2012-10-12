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

#include <comma/base/exception.h>
#include "./udp_reader.h"

namespace snark { 

udp_reader::udp_reader( unsigned short port )
    : socket_( service_ )
{
    socket_.open( boost::asio::ip::udp::v4() );
    boost::system::error_code error;
    socket_.set_option( boost::asio::ip::udp::socket::broadcast( true ), error );
    if( error ) { COMMA_THROW( comma::exception, "failed to set broadcast option on port " << port ); }
    socket_.set_option( boost::asio::ip::udp::socket::reuse_address( true ), error );
    if( error ) { COMMA_THROW( comma::exception, "failed to set reuse address option on port " << port ); }
    socket_.bind( boost::asio::ip::udp::endpoint( boost::asio::ip::udp::v4(), port ), error );
    if( error ) { COMMA_THROW( comma::exception, "failed to bind port " << port ); }

}

const char* udp_reader::read()
{
    boost::system::error_code error;
    std::size_t size = socket_.receive( boost::asio::buffer( packet_ ), 0, error );
    if( error || size == 0 ) { return NULL; }
    timestamp_ = boost::posix_time::microsec_clock::universal_time();
    return &packet_[0];
}

void udp_reader::close() { socket_.close(); }

const boost::posix_time::ptime& udp_reader::timestamp() const { return timestamp_; }

} // namespace snark {

