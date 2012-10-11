#ifndef SNARK_SENSORS_VELODYNE_PCAPREADER_H_
#define SNARK_SENSORS_VELODYNE_PCAPREADER_H_

#ifndef WIN32
#include <stdlib.h>
#endif
#include <pcap.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/noncopyable.hpp>

namespace snark {
    
/// a simple pcap wrapper
/// @todo move to a more appropriate place, once we figure out where
class pcap_reader : public boost::noncopyable
{
    public:
        /// constructor, open a pcap file, default stdin
        pcap_reader( const std::string& filename = "-" );
    
        /// destructor, close file
        ~pcap_reader();
    
        /// read and return pointer to the current packet; NULL, if end of file
        const char* read();
    
        /// close
        void close();
    
        /// return true, if end of file
        bool eof() const;
    
        /// return current timestamp
        boost::posix_time::ptime timestamp() const;
    
    private:
        char m_error[1024];
        ::pcap_t* m_handle;
        pcap_pkthdr m_header;
};

} 

#endif /*SNARK_SENSORS_VELODYNE_PCAPREADER_H_*/

