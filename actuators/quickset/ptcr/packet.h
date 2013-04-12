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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#ifndef SNARK_ACTUATORS_QUICKSET_ptcr_PACKET_H_
#define SNARK_ACTUATORS_QUICKSET_ptcr_PACKET_H_

#include <comma/packed/byte.h>
#include <comma/packed/little_endian.h>
#include <comma/packed/struct.h>

namespace snark { namespace quickset { namespace ptcr {

struct constants
{
    static const unsigned char stx = 0x02;
    static const unsigned char etx = 0x03;
    static const unsigned char ack = 0x06;
    static const unsigned char nak = 0x15;
    static const unsigned char esc = 0x1B;
};
  
template < typename B >
struct header : public comma::packed::packed_struct< header< B >, 3 >
{
    comma::packed::byte type;
    comma::packed::byte address;
    comma::packed::const_byte< B::id > id;
    
    header( unsigned char t ) { type = t; address = 0; }
};

template < typename B >
struct lrc : public comma::packed::packed_struct< lrc< B >, 1 >
{
    comma::packed::byte value;
    
    bool ok() const;
    void set();
};

template < typename B >
struct footer : public comma::packed::packed_struct< footer< B >, 2 >
{
    lrc< B > footer_lrc;
    comma::packed::const_byte< 0x03 > etx;
};

template < typename B >
struct packet : public comma::packed::packed_struct< packet< B >, 3 + 2 + B::size >
{
    header< B > packet_header;
    B body;
    footer< B > packet_footer;

    packet() : packet_header( constants::stx ) {}
    packet( unsigned char type ) : packet_header( type ) {}
    packet( const B& body ) : packet_header( constants::stx ), body( body ) { packet_footer.footer_lrc.set(); }
    packet( const B& body, unsigned char type ) : packet_header( type ), body( body ) { packet_footer.footer_lrc.set(); }
};

inline static unsigned char get_lrc( const char* begin, const char* end )
{
    unsigned char c = 0;
    for( const char* p = begin; p < end; ++p ) { c ^= static_cast< unsigned char >( *p ); }
    return c;
}

template < typename B >
inline void lrc< B >::set() { value = get_lrc( value.data() - B::size - 1, value.data() ); }

template < typename B >
inline bool lrc< B >::ok() const
{
    return value() == get_lrc( value.data() - B::size - 1, value.data() );
}

} } } // namespace snark { namespace quickset { namespace ptcr {

#endif // #ifndef SNARK_ACTUATORS_QUICKSET_ptcr_PACKET_H_
