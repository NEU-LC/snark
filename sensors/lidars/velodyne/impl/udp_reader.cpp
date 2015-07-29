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


#include <comma/base/exception.h>
#include "udp_reader.h"

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

