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

#ifndef SNARK_SENSORS_VELODYNE_THIN_READER_H_
#define SNARK_SENSORS_VELODYNE_THIN_READER_H_

#ifndef WIN32
#include <stdlib.h>
#endif
#include <snark/sensors/velodyne/thin/thin.h>
#include <snark/timing/time.h>

namespace snark {

/// reader for thinned velodyne data    
class thin_reader : public boost::noncopyable
{
    public:
        const char* read()
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
            m_timestamp = boost::posix_time::ptime( snark::timing::epoch, boost::posix_time::seconds( static_cast< long >( seconds ) ) + boost::posix_time::microseconds( nanoseconds / 1000 ) );
            velodyne::thin::deserialize( m_packet, m_buf + timeSize );
            return reinterpret_cast< char* >( &m_packet );
        }

        void close() {}

        boost::posix_time::ptime timestamp() const { return m_timestamp; }

    private:
        enum { timeSize = 12 };
        char m_buf[ velodyne::thin::maxBufferSize + timeSize ];
        velodyne::packet m_packet;
        boost::posix_time::ptime m_timestamp;
};

} 

#endif // SNARK_SENSORS_VELODYNE_THIN_READER_H_
