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

#include <comma/base/types.h>
#include <comma/packed/byte.h>
#include <comma/packed/bits.h>
#include <comma/packed/struct.h>

namespace snark { namespace trimble { namespace bd9xx {

enum { stx = 0x02, etx = 0x03 };

struct status
{
    unsigned char reserved_0: 1,
                  battery_low: 1,
                  reserved_1: 1,
                  roving: 1,
                  reserved_2: 4;
};

/// http://www.trimble.com/OEM_ReceiverHelp/v4.91/en/default.html#DataCollectorFormatPackets.html
struct header : public comma::packed::packed_struct< header, 4 >
{
    comma::packed::const_byte< bd9xx::stx > stx;
    comma::packed::bits< bd9xx::status > status;
    comma::packed::uint8 type;
    comma::packed::uint8 length; // including header (unusually)
};

struct trailer : public comma::packed::packed_struct< trailer, 2 >
{
    comma::packed::uint8 checksum;
    comma::packed::const_byte< bd9xx::etx > etx;
};

struct packet_with_no_data : public comma::packed::packed_struct< packet_with_no_data, bd9xx::header::size + bd9xx::trailer::size >
{
    bd9xx::header header;
    bd9xx::trailer trailer;
};

template < typename Data >
struct packet : public comma::packed::packed_struct< packet< Data >, bd9xx::header::size + Data::size + bd9xx::trailer::size >
{
    bd9xx::header header;
    Data data;
    bd9xx::trailer trailer;
};
    
} } } // namespace snark { namespace trimble { namespace bd9xx {

#endif // SNARK_NAVIGATION_TRIMBLE_BD9XX_PACKET_H_
