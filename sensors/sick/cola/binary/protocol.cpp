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

/// @author andrew hill
/// @author vsevolod vlaskine (v.vlaskine@acfr.usyd.edu.au)

#include <comma/base/exception.h>
#include "protocol.h"

namespace snark { namespace sick { namespace cola { namespace binary {

void protocol::read_packet( std::istream& is, std::vector< char >& buf )
{
    //while( true ) // todo: resync
    { 
        buf.resize( header::size );
        is.read( &buf[0], header::size ); // todo: if resync required, read 4 bytes first
        const header* h = reinterpret_cast< const header* >( &buf[0] );
        if( !h->valid() ) { COMMA_THROW( comma::exception, "resync: todo" ); }
        buf.resize( header::size + h->length() + 1 );
        is.read( &buf[0] + header::size, buf.size() - header::size );
        // todo: if crc invalid, resync
        return;
    }
}

template <> void protocol::handle_< payloads::set_access_mode::request >( const cola::binary::packet< payloads::set_access_mode::request >& p )
{
    // todo
}

void protocol::handle_packet( const char* p )
{
    const std::string& type = reinterpret_cast< const cola::binary::header* >( p )->type();
    if( type == "SetAccessType" ) { handle_< payloads::set_access_mode::request >( p ); }
    // etc
}
    
} } } } // namespace snark {  namespace sick { namespace cola { namespace binary {
