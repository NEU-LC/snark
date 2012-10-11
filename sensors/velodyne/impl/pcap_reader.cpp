#include <comma/base/exception.h>
#include <snark/timing/time.h>
#include "./pcap_reader.h"

namespace snark {
pcap_reader::pcap_reader( const std::string& filename )
    : m_handle( ::pcap_open_offline( filename.c_str(), m_error ) )
{
    #ifdef WIN32
    if( filename == "-" ) { _setmode( _fileno( stdin ), _O_BINARY ); }
    #endif
    if( m_handle == NULL ) { COMMA_THROW( comma::exception, "failed to open pcap file " << filename ); }
}

pcap_reader::~pcap_reader() { close(); }

const char* pcap_reader::read() { return reinterpret_cast< const char* >( ::pcap_next( m_handle, &m_header ) ); }

boost::posix_time::ptime pcap_reader::timestamp() const
{
    return boost::posix_time::ptime( snark::timing::epoch, boost::posix_time::seconds( m_header.ts.tv_sec ) + boost::posix_time::microseconds( m_header.ts.tv_usec ) );
}

void pcap_reader::close()
{
    if( m_handle ) { ::pcap_close( m_handle ); }
}

} 

