#ifndef SNARK_SENSORS_VELODYNE_IMPL_STREAMTRAITS_H_
#define SNARK_SENSORS_VELODYNE_IMPL_STREAMTRAITS_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include "./pcap_reader.h"
#include "./proprietary_reader.h"

namespace snark {  namespace velodyne { namespace impl {

template < typename S >
struct stream_traits
{
    //static const char* read( S& s, std::size_t size ) { return s.read( size ); }
    static const char* read( S& s, std::size_t size ) { return s.read(); }
    
    //static boost::posix_time::ptime timestamp( const S& ) { return boost::posix_time::microsec_clock::local_time(); }
    static boost::posix_time::ptime timestamp( const S& s ) { return s.timestamp(); }
    
    static void close( S& s ) { s.close(); }
};

template <>
struct stream_traits< proprietary_reader >
{
    static const char* read( proprietary_reader& s, std::size_t ) { return s.read(); }
    
    static boost::posix_time::ptime timestamp( const proprietary_reader& s ) { return s.timestamp(); }
    
    static void close( proprietary_reader& s ) { s.close(); }
};

template <>
struct stream_traits< pcap_reader >
{
    static const char* read( pcap_reader& s, std::size_t size )
    {
        const char* r = s.read();
        if( r == NULL ) { return NULL; }
        return r + 42; // skip UDP header
    }
    
    static boost::posix_time::ptime timestamp( const pcap_reader& s ) { return s.timestamp(); }
    
    static void close( pcap_reader& s ) { s.close(); }
};

} } } // namespace snark {  namespace velodyne { namespace impl {

#endif // SNARK_SENSORS_VELODYNE_IMPL_STREAMTRAITS_H_
