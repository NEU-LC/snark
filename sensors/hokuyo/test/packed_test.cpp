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
#include <list>
#include <snark/timing/time.h>
#include <gtest/gtest.h>
#include <comma/csv/stream.h>
#include "../message.h"
#include "../traits.h"

namespace hok = snark::hokuyo;
namespace packed = comma::packed;


TEST( hokuyo_packed, requests )
{
    hok::header header;
    hok::sequence_string msg_id;

    hok::request_gd gd;
    hok::request_gd ge( true );
    hok::request_md md;
    hok::request_md me( true );
    hok::reply_gd< 1080 > gd_data;
    hok::reply_ge< 1080 > ge_data;
    hok::reply_md md_response;
    hok::reply_md_data< 1080 > md_data;
    hok::reply_me_data< 1080 > me_data;
    
    gd.header.start_step = 0;
    gd.header.end_step = 100;
    gd.message_id.seq_num = 999;
    
    const std::string gd_request( "GD0000010000;GD00000999\n" );
    
    std::string result( gd.data(), hok::request_gd::size );
    EXPECT_EQ( gd_request, result );

    const std::string ge_request( "GE0000108000;GE00001111\n" );
    ge.header.start_step = 0;
    ge.header.end_step = 1080;
    ge.message_id.seq_num = 1111;
    result = std::string( ge.data(), hok::request_gd::size );
    EXPECT_EQ( ge_request, result );
    
    md.header.start_step = 5;
    md.header.end_step = 10;
    md.num_of_scans = 2; // two scans only
    md.message_id.seq_num = 222;
    result = std::string( md.data(), hok::request_md::size );
    EXPECT_EQ( "MD0005001000002;MD00000222\n", result );
    
    me.header.start_step = 5;
    me.header.end_step = 10;
    me.num_of_scans = 2; // two scans only
    me.message_id.seq_num = 222;
    result = std::string( me.data(), hok::request_md::size );
    EXPECT_EQ( "ME0005001000002;ME00000222\n", result );
    
    md_response.request = md;
    md_response.status.sum = 'P';
    result = std::string( md_response.data(), hok::reply_md::size );
    EXPECT_EQ( "MD0005001000002;MD00000222\n00P\n\n", result );
    
    
}

TEST( hokuyo_packed, scip_response )
{
    hok::request_gd gd;
    gd.header.start_step = 0;
    gd.header.end_step = 10;
    gd.message_id.seq_num = 999;
    
    const std::string gd_request( "GD0000001000;GD00000999\n" );
    // GD reponse for request with steps 0-10: 11 data points
    const std::string response =  "GD0000001000;GD00000999\n00P\nG]\\VF\n06306J06_07407607106i06i06i075070n\n\n";
    
    const hok::reply_gd< 11 >* gd_reply = reinterpret_cast< const hok::reply_gd< 11 >* >( &response[0] );
    
    EXPECT_EQ( "00P", std::string( gd_reply->status.data(), hok::status_t::size ) );
    EXPECT_TRUE( hok::scip_verify_checksum( std::string( gd_reply->status.data(), hok::status_t::size ) ) );
    EXPECT_EQ( "00", std::string( gd_reply->status.status.data(), 2 ) );
    EXPECT_EQ( "G]\\VF", std::string( gd_reply->timestamp.data(), hok::timestamp_t::size ) );
//     EXPECT_EQ( 0, gd_reply->status.status() );
    EXPECT_EQ( 6216486, gd_reply->timestamp.timestamp() );
    EXPECT_EQ( 'F', gd_reply->timestamp.sum );
    /// Line feeds are included in the data, remove it to verify checksum, as that value is the last 'n'
    EXPECT_EQ( "06306J06_07407607106i06i06i075070n", std::string( gd_reply->data.raw_data.data(), hok::distance_data< 11 >::value-1 ) );
    
    hok::distance_data< 11 >::rays rays;
    gd_reply->data.get_values( rays );
    
    // std::cerr << "11 values are : ";
    // for( std::size_t i=0; i<rays.steps.size(); ++i )
    // {
    //     std::cerr << rays.steps[i]() << ' ';
    // }
    // std::cerr << std::endl;
    
    /// Why fails checksum
    //     std::cerr << "11 values are : ";
    //     comma::csv::output_stream< hok::distance_data< 11 > > oss( std::cerr );
    //     oss.write( gd_reply->data );
    
    /// 100 steps/rays
    gd.header.end_step = 100;
    
    const char* reply101 = "GD0000010000;GD00000999\n00P\n\\0[Vm"
        "\n0Mm0MV0Jc0If0I30HX0HL0H;0G`0Gb0Gm0GV0GT0G;0FQ0FB0F<0F40Eo0En0E`0a"
        "\nEW0ER0ER0E40E90E40E80E<0E90E90E=0E>0E>0ED0EI0EQ0EX0EX0EW0EY0E_0Ef"
        "\ng0F80F>0FA0FC0FA0F?0FK0FN0F[0F^0F[0Fa0G50Gb?om?om?om?om?om?om?omi"
        "\n?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?f"
        "\nom?om0I\\0IR0I[0I\\0I`?om?om?om?om?om0J60J60In0Ie5\n\n";
    
    
    const hok::reply_gd< 101 >* reply = reinterpret_cast< const hok::reply_gd< 101 >* >( reply101 );
    EXPECT_EQ( "00P", std::string( reply->status.data(), hok::status_t::size ) );
    EXPECT_TRUE( hok::scip_verify_checksum( std::string( reply->status.data(), hok::status_t::size ) ) );
    EXPECT_EQ( "00", std::string( reply->status.status.data(), 2 ) );
    EXPECT_EQ( "\\0[Vm", std::string( reply->timestamp.data(), hok::timestamp_t::size ) );
    EXPECT_TRUE( hok::scip_verify_checksum( "0Mm0MV0Jc0If0I30HX0HL0H;0G`0Gb0Gm0GV0GT0G;0FQ0FB0F<0F40Eo0En0E`0a" ) );
    EXPECT_TRUE( hok::scip_verify_checksum( "EW0ER0ER0E40E90E40E80E<0E90E90E=0E>0E>0ED0EI0EQ0EX0EX0EW0EY0E_0Ef" ) );
    EXPECT_TRUE( hok::scip_verify_checksum( "g0F80F>0FA0FC0FA0F?0FK0FN0F[0F^0F[0Fa0G50Gb?om?om?om?om?om?om?omi" ) );
    EXPECT_TRUE( hok::scip_verify_checksum( "?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?f" ) );
    EXPECT_TRUE( hok::scip_verify_checksum( "om?om0I\\0IR0I[0I\\0I`?om?om?om?om?om0J60J60In0Ie5" ) );

    hok::distance_data< 101 >::rays rays101;
    reply->data.get_values( rays101 );
    
    // std::cerr << "11 values are : ";
    // for( std::size_t i=0; i<rays101.steps.size(); ++i )
    // {
    //     std::cerr << rays101.steps[i]() << ' ';
    // }
    // std::cerr << std::endl;

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
//     hok::distance_data< 4 > data;
//     hok::distance_data< 4 >::rays points;
//     
//     data.get_values( points );
//     
//     hok::di_data< 4 > data2;
//     hok::di_data< 4 >::rays points2;
//     
//     data2.get_values( points2 );
}

