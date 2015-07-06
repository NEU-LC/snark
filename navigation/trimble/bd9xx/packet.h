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

/// @author vsevolod vlaskine

#ifndef SNARK_NAVIGATION_TRIMBLE_BD9XX_PACKET_H_
#define SNARK_NAVIGATION_TRIMBLE_BD9XX_PACKET_H_

#include <boost/array.hpp>
#include <comma/base/types.h>
#include <comma/packed/byte.h>
#include <comma/packed/bits.h>
#include <comma/packed/struct.h>

namespace snark { namespace trimble { namespace bd9xx {

/// http://www.trimble.com/OEM_ReceiverHelp/v4.91/en/default.html#DataCollectorFormatPackets.html
    
enum { stx = 0x02, etx = 0x03 };

struct status
{
    unsigned char reserved_0: 1,
                  battery_low: 1,
                  reserved_1: 1,
                  roving: 1,
                  reserved_2: 4;
                  
    status() : reserved_0( 0 ), battery_low( 0 ), reserved_1( 0 ), roving( 0 ), reserved_2( 0 ) {}
};

struct header : public comma::packed::packed_struct< header, 4 >
{
    comma::packed::const_byte< bd9xx::stx > stx;
    comma::packed::bits< bd9xx::status > status;
    comma::packed::uint8 type;
    comma::packed::uint8 length;
    
    header( unsigned char t = 0, unsigned char s = 0, unsigned char l = 0 ) { type = t; status = s; length = l; }
    unsigned char checksum() const { return *status.data() + *type.data(); }
    bool valid() const { return stx() == stx.default_value(); }
};

struct trailer : public comma::packed::packed_struct< trailer, 2 >
{
    comma::packed::uint8 checksum;
    comma::packed::const_byte< bd9xx::etx > etx;
    
    bool valid() const { return etx() == etx.default_value(); }
};

struct packet : public boost::array< char, 4 + 255 + 2 >
{
    const bd9xx::header& header() const;
    
    bd9xx::header& header();
    
    const bd9xx::trailer& trailer() const;
    
    bd9xx::trailer& trailer();
    
    const char* body() const;
    
    char* body();
    
    unsigned char checksum() const;
    
    bool valid() const;
    
    template < typename T > const T& as() const { return *reinterpret_cast< const T* >( &( this->operator[](0) ) ); }
    
    template < typename T > T& as() { return *reinterpret_cast< T* >( &( this->operator[](0) ) ); }
};

template < unsigned char Type >
struct simple_packet : public comma::packed::packed_struct< simple_packet< Type >, bd9xx::header::size + bd9xx::trailer::size >
{
    enum { type = Type };
    
    bd9xx::header header;
    bd9xx::trailer trailer;
    
    simple_packet( unsigned char status = 0 ) : header( type, status ) { trailer.checksum = header.checksum(); }
    
    bool valid() const { return header.valid() && trailer.valid(); }
};

template < unsigned char Type, typename Body >
struct fixed_packet : public comma::packed::packed_struct< fixed_packet< Type, Body >, bd9xx::header::size + Body::size + bd9xx::trailer::size >
{
    enum { type = Type };
    
    bd9xx::header header;
    Body body;
    bd9xx::trailer trailer;
    
    fixed_packet() : header( Type, 0, Body::size ) {}
    unsigned char checksum() const;
    void set_checksum();
    bool valid() const;
};

template < unsigned char Type, typename Body >
inline unsigned char fixed_packet< Type, Body >::checksum() const { return reinterpret_cast< const packet* >( this )->checksum(); }

template < unsigned char Type, typename Body >
inline bool fixed_packet< Type, Body >::valid() const { return header.valid() && trailer.valid() && checksum() == trailer.checksum(); }

template < unsigned char Type, typename Body >
inline void fixed_packet< Type, Body >::set_checksum() { trailer.checksum = checksum(); }
    
} } } // namespace snark { namespace trimble { namespace bd9xx {

#endif // SNARK_NAVIGATION_TRIMBLE_BD9XX_PACKET_H_
