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
