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


#ifndef WIN32
#include <stdlib.h>
#endif

#include <iostream>
#include <snark/timing/time.h>
#include <gtest/gtest.h>
#include "../message.h"

namespace hok = snark::hokuyo;
namespace packed = comma::packed;


TEST( hokuyo_packed, header )
{
    hok::header header;
    hok::sequence_string msg_id;

    hok::request_gd gd;
    hok::request_md md;
    hok::reply_gd< 1080 > gd_data;
    hok::reply_ge< 1080 > ge_data;
    hok::reply_md md_reponse;
    hok::reply_md_data< 1080 > md_data;
    hok::reply_me_data< 1080 > me_data;
}

TEST( hokuyo_encoding, scip_format )
{
    const std::string data( "1Dh");

    const packed::scip_3chars_t* h3 = reinterpret_cast< const packed::scip_3chars_t* >( &data[0] );

    comma::uint32 value = 5432;
    EXPECT_EQ( value, (*h3)() );
    packed::scip_3chars_t h3_data;
    h3_data = value;
    std::string out( h3_data.data(), 3 );
    EXPECT_EQ( data, out );


    const std::string data4 = "m2@0";
    const comma::uint32 value4 = 16000000;
    const packed::scip_4chars_t* h4 = reinterpret_cast< const packed::scip_4chars_t* >( &data4[0] );
    EXPECT_EQ( value4, (*h4)() );

    packed::scip_4chars_t h4_data;
    h4_data = value4;
    std::string out4( h4_data.data(), 4 );
    EXPECT_EQ( data4, out4 );
}

TEST( hokuyo, strip_checksum )
{
    hok::distance_data< 4 > data;
    hok::distance_data< 4 >::rays points;
    
    data.get_values( points );
    
    hok::di_data< 4 > data2;
    hok::di_data< 4 >::rays points2;
    
    data2.get_values( points2 );
}

