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

#ifdef WIN32
#include <fcntl.h>
#include <io.h>
#endif
#include "thin_reader.h"

namespace snark {

snark::thin_reader::thin_reader() : is_new_scan_( true )
{
    #ifdef WIN32
    _setmode( _fileno( stdin ), _O_BINARY );
    #endif
}

const char* thin_reader::read()
{
    if( !std::cin.good() || std::cin.eof() ) { return NULL; }
    comma::uint16 size;
    std::cin.read( reinterpret_cast< char* >( &size ), 2 );
    if( std::cin.gcount() < 2 ) { return NULL; }
    std::cin.read( m_buf, size );
    if( std::cin.gcount() < size ) { return NULL; }
    comma::int64 seconds;
    comma::int32 nanoseconds;
    ::memcpy( &seconds, m_buf, sizeof( comma::int64 ) );
    ::memcpy( &nanoseconds, m_buf + sizeof( comma::int64 ), sizeof( comma::int32 ) );
    m_timestamp = boost::posix_time::ptime( snark::timing::epoch, boost::posix_time::seconds( static_cast< long >( seconds ) ) + boost::posix_time::microseconds( static_cast< long >( nanoseconds / 1000 ) ) );
    comma::uint32 scan = velodyne::thin::deserialize( m_packet, m_buf + timeSize );
    is_new_scan_ = is_new_scan_ || !last_scan_ || *last_scan_ != scan; // quick and dirty; keep it set until we clear it in is_new_scan()
    last_scan_ = scan;
    return reinterpret_cast< char* >( &m_packet );
}

void thin_reader::close() {}

boost::posix_time::ptime thin_reader::timestamp() const { return m_timestamp; }

bool thin_reader::is_new_scan()
{
    bool r = is_new_scan_;
    is_new_scan_ = false;
    return r;
}

} // namespace snark {
