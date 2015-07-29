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

#ifndef SNARK_SENSORS_SICK_COLA_BINARY_PACKET_H_
#define SNARK_SENSORS_SICK_COLA_BINARY_PACKET_H_

#include <comma/packed/byte.h>
#include <comma/packed/little_endian.h>
#include <comma/packed/big_endian.h>
#include <comma/packed/string.h>
#include <comma/packed/struct.h>

namespace snark { namespace sick { namespace cola { namespace binary {

struct header : public comma::packed::packed_struct< header, 8 >
{
    comma::packed::net_uint32 sentinel;
    comma::packed::net_uint32 length;

    bool valid() const;
    std::string type() const;
    static std::string type( const char* packet );
};

template < unsigned int TypeFieldSize >
struct body_header : public comma::packed::packed_struct< body_header< TypeFieldSize >, 4 + TypeFieldSize + 1 >
{
    comma::packed::string< 3 > command_type;
    comma::packed::const_byte< ' ' > space1;
    comma::packed::string< TypeFieldSize, ' ' > type;
    comma::packed::const_byte< ' ' > space2;
};

template < typename P >
struct body : public comma::packed::packed_struct< body< P >, 4 + P::type_field_size + 1 + P::size >
{
    typedef body_header< P::type_field_size > header_t;
    header_t header;
    P payload;
};

template < typename P >
struct packet : public comma::packed::packed_struct< packet< P >, header::size + body< P >::size + 1 >
{
    binary::header header;
    binary::body< P > body;
    comma::packed::byte crc;

    bool valid() const;
};

} } } } // namespace snark {  namespace sick { namespace cola { namespace binary {

#endif // #ifndef SNARK_SENSORS_SICK_COLA_BINARY_PACKET_H_
