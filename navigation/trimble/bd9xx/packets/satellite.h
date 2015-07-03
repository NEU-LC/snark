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

#ifndef SNARK_NAVIGATION_TRIMBLE_BD9XX_PACKETS_SATELLITE_H_
#define SNARK_NAVIGATION_TRIMBLE_BD9XX_PACKETS_SATELLITE_H_

#include <comma/base/exception.h>
#include <comma/packed/string.h>
#include <comma/packed/big_endian.h>
#include "../packet.h"

namespace snark { namespace trimble { namespace bd9xx { namespace packets {

struct satellite // getsvdata
{
    enum subtype
    {
          sv_flags_deprecated = 0
        , gps_ephemeris = 1
        , gps_almanac = 2
        , ion_utc_data = 3
        , disable_satellite_deprecated = 4
        , enable_satellite_deprecated = 5
        , extended_gps_almanac = 7
        , glonass_almanac = 8
        , glonass_ephemeris = 9
        , galileo_almanac = 11
        , galileo_ephemeris = 12
        , qzss_almanac = 14
        , qzss_ephemeris = 16
        , sv_enable_disable_ignore_health = 20
        , beidou_almanac = 21
        , beidou_ephemeris = 22
    };
    // todo
    
    struct data // todo: correct name?
    {
        struct request
        {
            struct data : public comma::packed::packed_struct< data, 3 >
            {
                comma::packed::uint8 subtype;
                comma::packed::uint8 sv_prn_number; // todo: see documentation
                comma::packed::uint8 flags; // todo: see documentation
            };
            
            typedef bd9xx::fixed_packet< 0x54, data > packet;
        };
        
        struct response; // todo
    };
    
    struct control // todo: correct name?
    {
        struct request
        {
            struct data : public comma::packed::packed_struct< data, 4 >
            {
                comma::packed::uint8 subtype;
                comma::packed::uint8 sv_prn_number; // todo: see documentation
                comma::packed::uint8 satellite_type; // todo: see documentation
                comma::packed::uint8 mode; // todo: see documentation
            };
            
            typedef bd9xx::fixed_packet< 0x54, data > packet;
        };
        
        struct response; // todo
    };    
};
    
} } } } // namespace snark { namespace trimble { namespace bd9xx { namespace packets {

#endif // SNARK_NAVIGATION_TRIMBLE_BD9XX_PACKETS_SATELLITE_H_
