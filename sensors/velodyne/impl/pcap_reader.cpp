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

