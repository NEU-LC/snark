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
#include <comma/base/types.h>
#include <snark/timing/time.h>
#include "./proprietary_reader.h"

#ifdef WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace snark {
    
proprietary_reader::proprietary_reader( const std::string& filename )
    : m_offset( 0 )
    , m_end( 0 )
{
    if( filename == "-" )
    {
        #ifdef WIN32
        if( filename == "-" ) { _setmode( _fileno( stdin ), _O_BINARY ); }
        #endif
        m_istream = &std::cin;
    }
    else
    {
        m_ifstream.reset( new std::ifstream( filename.c_str(), std::ios::binary | std::ios::in ) );
        if( m_ifstream->bad() ) { COMMA_THROW( comma::exception, "failed to open file " << filename ); }
        m_istream = m_ifstream.get();
    }
}

proprietary_reader::~proprietary_reader() { close(); }

const char* proprietary_reader::read() // quick and dirty
{
    static const boost::array< char, 2 > start = {{ -78, 85 }}; // see QLib::Bytestreams::GetDefaultstartDelimiter()
    static const boost::array< char, 2 > end = {{ 117, -97 }}; // see QLib::Bytestreams::GetDefaultstartDelimiter()
    while(    m_offset >= m_end
           || ::memcmp( &m_buffer[ m_offset ], &start[0], 2 ) != 0
           || ::memcmp( &m_buffer[ m_offset + packetSize - 2 ], &end[0], 2 ) != 0 )
    {
        if( m_offset + packetSize > m_end )
        {
            if( m_istream->bad() || m_istream->eof() ) { return NULL; }
            std::size_t size = m_buffer.size();
            std::size_t len = 0;
            if( m_offset < m_end )
            {
                len = m_end - m_offset;
                ::memcpy( &m_buffer[0], &m_buffer[m_offset], len );
                size -= len;
            }
            m_istream->read( &m_buffer[len], size );
            if( m_istream->gcount() <= 0 ) { return NULL; }
            m_end = m_istream->gcount() + len;
            if( m_end < packetSize ) { return NULL; }
            m_offset = 0;
        }
        else
        {
            ++m_offset;
        }
    }
    char* t = &m_buffer[m_offset] + headerSize;
    comma::uint64 seconds;
    comma::uint32 nanoseconds;
    ::memcpy( &seconds, t, 8 );
    ::memcpy( &nanoseconds, t + 8, 4 );
    m_timestamp = boost::posix_time::ptime( snark::timing::epoch, boost::posix_time::seconds( seconds ) + boost::posix_time::microseconds( nanoseconds / 1000 ) );
    m_offset += packetSize;
    return t + timestampSize;
}

boost::posix_time::ptime proprietary_reader::timestamp() const { return m_timestamp; }

void proprietary_reader::close() { if( m_ifstream ) { m_ifstream->close(); } }

} 

