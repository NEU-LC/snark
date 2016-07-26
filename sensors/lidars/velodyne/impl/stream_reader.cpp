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
#include "stream_reader.h"
#include "../../../../timing/time.h"
#ifdef WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace snark {

stream_reader::stream_reader( std::istream& is ) : istream_( is ), m_epoch( timing::epoch )
{
    #ifdef WIN32
    if( is == std::cin ) { _setmode( _fileno( stdin ), _O_BINARY ); }
    #endif
}

stream_reader::stream_reader( const std::string& filename )
    : ifstream_( new std::ifstream( &filename[0], std::ios::binary ) )
    , istream_( *ifstream_ )
    , m_epoch( timing::epoch )
{
}

void stream_reader::close() { if( ifstream_ ) { ifstream_->close(); } }

stream_reader::~stream_reader() { close(); }

const char* stream_reader::read()
{
    istream_.read( reinterpret_cast< char* >( &m_microseconds ), sizeof( m_microseconds ) );
    istream_.read( m_packet.data(), payload_size );
    if( istream_.bad() || istream_.eof() ) { return NULL; }
    comma::uint64 seconds = m_microseconds / 1000000; //to avoid time overflow on 32bit systems with boost::posix_time::microseconds( m_microseconds ), apparently due to a bug in boost
    comma::uint64 microseconds = m_microseconds % 1000000;
    m_timestamp = m_epoch + boost::posix_time::seconds( seconds ) + boost::posix_time::microseconds( microseconds ); // todo: encapsulate time conversion (use a wrapper from comma)
    return &m_packet[0];
}

const boost::posix_time::ptime& stream_reader::timestamp() const { return m_timestamp; }

} // namespace snark {

