#ifndef SNARK_SENSORS_VELODYNE_PROPRIETARYREADER_H_
#define SNARK_SENSORS_VELODYNE_PROPRIETARYREADER_H_

#ifndef WIN32
#include <stdlib.h>
#endif
#include <iostream>
#include <fstream>
#include <boost/array.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/noncopyable.hpp>
#include <boost/scoped_ptr.hpp>

namespace snark {

/// a simple pcap wrapper
/// @todo move to a more appropriate place, once we figure out where
class proprietary_reader : public boost::noncopyable
{
    public:
        /// constructor, open a file, default stdin
        proprietary_reader( const std::string& filename = "-" );
    
        /// destructor, close file
        ~proprietary_reader();
    
        /// read and return pointer to the current packet; NULL, if end of file
        const char* read();
    
        /// close
        void close();
    
        /// return current timestamp
        boost::posix_time::ptime timestamp() const;
    
    private:
        enum
        {
              headerSize = 16
            , timestampSize = 12
            , payload_size = 1206
            , footerSize = 4
            , packetSize = headerSize + timestampSize + payload_size + footerSize
            , packetNum = 1 // , packetNum = 10 : does not seem to make big impact on the i/o performance
        };
        boost::array< char, packetSize * packetNum > m_buffer;
        std::size_t m_offset;
        std::size_t m_end;
        boost::posix_time::ptime m_timestamp;
        boost::scoped_ptr< std::ifstream > m_ifstream;
        std::istream* m_istream;
};

} 

#endif /*SNARK_SENSORS_VELODYNE_PROPRIETARYREADER_H_*/
