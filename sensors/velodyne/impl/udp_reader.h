#ifndef SNARK_SENSORS_VELODYNE_UDPREADER_H_
#define SNARK_SENSORS_VELODYNE_UDPREADER_H_

#ifndef WIN32
#include <stdlib.h>
#endif
#include <boost/array.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/noncopyable.hpp>

namespace snark { 

/// udp reader
class udp_reader : public boost::noncopyable
{
    public:
        /// constructor
        udp_reader( unsigned short port );

        /// read and return pointer to the current packet; NULL, if end of file
        const char* read();

        /// close
        void close();

        /// return current timestamp
        const boost::posix_time::ptime& timestamp() const;

    private:
        boost::asio::io_service service_;
        boost::asio::ip::udp::socket socket_;
        boost::array< char, 2000 > packet_; // way greater than velodyne packet
        boost::posix_time::ptime timestamp_;
};

} // namespace snark {

#endif /*SNARK_SENSORS_VELODYNE_UDPREADER_H_*/

