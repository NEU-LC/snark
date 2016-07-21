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


#include <comma/base/exception.h>
#include "../../../../timing/time.h"
#include "pcap_reader.h"

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

