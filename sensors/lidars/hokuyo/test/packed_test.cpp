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


#ifndef WIN32
#include <stdlib.h>
#endif

#include <iostream>
#include <list>
#include <boost/asio/streambuf.hpp>
#include "../../../../timing/time.h"
#include <gtest/gtest.h>
#include <comma/csv/stream.h>
#include "../message.h"
#include "../traits.h"

namespace hok = snark::hokuyo;
namespace packed = comma::packed;

TEST( hokuyo_encoding, format )
{
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
    
    {
        const std::string data( "06H");
    
        const packed::scip_3chars_t* h3 = reinterpret_cast< const packed::scip_3chars_t* >( &data[0] );
    
        comma::uint32 value = 408;
        EXPECT_EQ( value, (*h3)() );
        packed::scip_3chars_t h3_data;
        h3_data = value;
        std::string out( h3_data.data(), 3 );
        EXPECT_EQ( data, out );
        
    }
}

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

TEST( hokuyo_packed, scip_gd_response )
{
    hok::request_gd gd;
    gd.header.start_step = 0;
    gd.header.end_step = 10;
    gd.message_id.seq_num = 999;
    
    const std::string gd_request( "GD0000001000;GD00000999\n" );
    // GD reponse for request with steps 0-10: 11 data points
    const std::string response =  "GD0000001000;GD00000999\n00P\nG]\\VF\n06306J06_07407607106i06i06i075070n\n\n";
    
    const hok::reply_gd< 11 >* gd_reply = reinterpret_cast< const hok::reply_gd< 11 >* >( &response[0] );
    
    EXPECT_EQ( "00P", std::string( gd_reply->header.status.data(), hok::status_t::size ) );
    EXPECT_TRUE( hok::verify_checksum( std::string( gd_reply->header.status.data(), hok::status_t::size ) ) );
    EXPECT_EQ( "00", std::string( gd_reply->header.status.status.data(), 2 ) );
    EXPECT_EQ( "G]\\VF", std::string( gd_reply->header.timestamp.data(), hok::timestamp_t::size ) );
//     EXPECT_EQ( 0, gd_reply->header.status.status() );
    EXPECT_EQ( 6216486, gd_reply->header.timestamp() );
    EXPECT_EQ( 'F', gd_reply->header.timestamp.sum );
    /// Line feeds are included in the data, remove it to verify checksum, as that value is the last 'n'
    EXPECT_EQ( "06306J06_07407607106i06i06i075070n", std::string( gd_reply->encoded.raw_data.data(), hok::distance_data< 11 >::value-1 ) );
    
    hok::distance_data< 11 >::rays rays;
    gd_reply->encoded.get_values( rays );
    
    {
        std::ostringstream ss;
        for( std::size_t i=0; i<rays.steps.size(); ++i )
        {
            ss << rays.steps[i]() << ' ';
        }
        // distances in mm
        EXPECT_EQ( "387 410 431 452 454 449 441 441 441 453 448 ", ss.str() );
    }
    
    {
        std::string line;
        static comma::csv::ascii< hok::distance_data< 11 >::points_t > ascii;
        EXPECT_EQ( "387,410,431,452,454,449,441,441,441,453,448", ascii.put( rays.steps, line ) );
        
    }
    
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
    EXPECT_EQ( "00P", std::string( reply->header.status.data(), hok::status_t::size ) );
    EXPECT_TRUE( hok::verify_checksum( std::string( reply->header.status.data(), hok::status_t::size ) ) );
    EXPECT_EQ( "00", std::string( reply->header.status.status.data(), 2 ) );
    EXPECT_EQ( "\\0[Vm", std::string( reply->header.timestamp.data(), hok::timestamp_t::size ) );
    EXPECT_TRUE( hok::verify_checksum( "0Mm0MV0Jc0If0I30HX0HL0H;0G`0Gb0Gm0GV0GT0G;0FQ0FB0F<0F40Eo0En0E`0a" ) );
    EXPECT_TRUE( hok::verify_checksum( "EW0ER0ER0E40E90E40E80E<0E90E90E=0E>0E>0ED0EI0EQ0EX0EX0EW0EY0E_0Ef" ) );
    EXPECT_TRUE( hok::verify_checksum( "g0F80F>0FA0FC0FA0F?0FK0FN0F[0F^0F[0Fa0G50Gb?om?om?om?om?om?om?omi" ) );
    EXPECT_TRUE( hok::verify_checksum( "?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?om?f" ) );
    EXPECT_TRUE( hok::verify_checksum( "om?om0I\\0IR0I[0I\\0I`?om?om?om?om?om0J60J60In0Ie5" ) );

    hok::distance_data< 101 >::rays rays101;
    reply->encoded.get_values( rays101 );
    
    // std::cerr << "11 values are : ";
    // for( std::size_t i=0; i<rays101.steps.size(); ++i )
    // {
    //     std::cerr << rays101.steps[i]() << ' ';
    // }
    // std::cerr << std::endl;

}

TEST( hokuyo_packed, scip_me_response )
{
    // asks for three scans
//  const char* request = "ME0100011000003;ME00000222\n";
    
    const char* me_response = "ME0100011000003;ME00000222\n"
    "00P\n";
    
    /// Note not showing one scan here - because we asked for 3 scans
    
    /// Note the 01 before the ';' means that remaining number of scans is 1
    const char* me_data = "ME0100011000001;ME00000222\n"
        "99b\n"
        "iSH[O\n"
        "0Id06k0J:06l0IY064?om000?om0000K10670Kd06G0M@06K0Mg06f0M^06X0Mh0G\n"
        "6Hn\n\n";
      
    const hok::reply_md* me = reinterpret_cast< const hok::reply_md* >( me_response ); 
    EXPECT_EQ( "00P", std::string( me->status.data(), hok::status_t::size ) );
    EXPECT_TRUE( hok::verify_checksum( std::string( me->status.data(), hok::status_t::size ) ) );
    
    
    boost::asio::streambuf buf;
    std::ostream oss( &buf ); 
    oss << me_data;
    std::istream iss( &buf );
    hok::reply_me_data< 11 > me_;
    EXPECT_TRUE( hok::read( me_, iss ) == 99 ); // testing hok::read()
    const hok::reply_me_data< 11 >* results = &me_;
    // const hok::reply_me_data< 11 >* results = reinterpret_cast< const hok::reply_me_data< 11 >* >( me_data );
    EXPECT_EQ( 1, results->header.request.num_of_scans() ); // 2nd scan of 3
    
    hok::di_data< 11 >::rays rays;
//     std::cerr << " steps 11 size: " << hok::reply_me_data< 11 >::size << std::endl;
//     std::cerr << " steps 11 data only size: " << hok::di_data< 11 >::value << std::endl;
    results->encoded.get_values( rays);
    
    {
        std::ostringstream ss;
        for( std::size_t i=0; i<11; ++i )
        {
            ss << rays.steps[i].distance() << ':' << rays.steps[i].intensity() << ' ';
        }
        // pairs of distance(mm) and intensity 
        EXPECT_EQ( "1652:443 1674:444 1641:388 65533:0 65533:0 1729:391 1780:407 1872:411 1911:438 1902:424 1912:408 ", ss.str() );
    }
    
    /// Note the 00 before the ';' means that remaining number of scans is 0
    me_data = "ME0100011000000;ME00000222\n"
        "99b\n"
        "9D7cG\n"
        "0Il0770Ij06e0J306<?om000?om000?om000?om0000ME06R0M\\06V0M_06X0Me0W\n"
        "6T:\n\n";
    results = reinterpret_cast< const hok::reply_me_data< 11 >* >( me_data );
    EXPECT_EQ( 0, results->header.request.num_of_scans() ); // 2nd scan of 3
    results->encoded.get_values( rays);
    
    {
        std::ostringstream ss;
        for( std::size_t i=0; i<11; ++i )
        {
            ss << rays.steps[i].distance() << ':' << rays.steps[i].intensity() << ' ';
        }
        // pairs of distance(mm) and intensity 
        EXPECT_EQ( "1660:455 1658:437 1667:396 65533:0 65533:0 65533:0 65533:0 1877:418 1900:422 1903:424 1909:420 ", ss.str() );
    }
}
TEST( hokuyo_packed, scip_md_response )
{
    // asks for two scans
    // const char* request = "MD0100011000002;MD00000222\n"
    
    /// Note the 01 before the ';' means that remaining number of scans is 1
    const char* md_data = "MD0100011000001;MD00000222\n"
    "99b\n"
    "Cl21B\n"
    "?om?om?om?om?om?om?om?om?om0JH0JJi\n\n"
    ;
      
    
    const hok::reply_md_data< 11 >* results = reinterpret_cast< const hok::reply_md_data< 11 >* >( md_data );
    EXPECT_EQ( 1, results->header.request.num_of_scans() ); // 1st scan of 2
    
    hok::distance_data< 11 >::rays rays;
    results->encoded.get_values( rays);
    
    {
        std::ostringstream ss;
        for( std::size_t i=0; i<11; ++i )
        {
            ss << rays.steps[i]() << ' ';
        }
        // distance data, 65533 is error
        EXPECT_EQ( "65533 65533 65533 65533 65533 65533 65533 65533 65533 1688 1690 ", ss.str() );
    }
    
    /// Note the 00 before the ';' means that remaining number of scans is 0
    md_data = "MD0100011000000;MD00000222\n"
        "99b\n"
        "Cl2J[\n"
        "?om?om?om?om?om?om?om?om?om0JI0JCc\n\n";
        
    results = reinterpret_cast< const hok::reply_md_data< 11 >* >( md_data );
    EXPECT_EQ( 0, results->header.request.num_of_scans() ); // 2nd scan of 3
    results->encoded.get_values( rays);
    
    {
        std::ostringstream ss;
        for( std::size_t i=0; i<11; ++i )
        {
            ss << rays.steps[i]() << ' ';
        }
        // pairs of distance(mm) and intensity 
        EXPECT_EQ( "65533 65533 65533 65533 65533 65533 65533 65533 65533 1689 1683 ", ss.str() );
    }
}

TEST( hokuyo_packed, scip_ge_response )
{
    hok::request_gd ge( true );
    ge.header.start_step = 0;
    ge.header.end_step = 10;
    ge.message_id.seq_num = 1111;
    
//     const char* request = "GE0000001000;GE00001111";
    
    const char* reply = "GE0000001000;GE00001111\n"
    "00P\n"
    "`mK3;\n"
    "0NC0<O0Mg0<Y0Je0<d0I[0<I0I90;00H]0:P0HD0:30H109l0H50;F0H40;X0Gj0^\n"
    ":cM\n\n";
    
    EXPECT_TRUE( hok::verify_checksum( std::string( "`mK3;" ) ) );
    EXPECT_TRUE( hok::verify_checksum( std::string( "0NC0<O0Mg0<Y0Je0<d0I[0<I0I90;00H]0:P0HD0:30H109l0H50;F0H40;X0Gj0^" ) ) );
    EXPECT_TRUE( hok::verify_checksum( std::string( ":cM" ) ) );

    boost::asio::streambuf buf;
    std::ostream oss( &buf ); 
    oss << reply;

    std::istream iss( &buf );
    hok::reply_ge< 11 > ge_reply;
    EXPECT_TRUE( hok::read( ge_reply, iss ) == 0 );
    // const hok::reply_ge< 11 >* ge_reply = reinterpret_cast< const hok::reply_ge< 11 >* >( reply );
    EXPECT_EQ( "00P", std::string( ge_reply.header.status.data(), hok::status_t::size ) );
    EXPECT_EQ( 0, ge_reply.header.status() );

    typedef hok::di_data< 11 > data_t;
    data_t::rays rays;
    ge_reply.encoded.get_values( rays );
    
}

TEST( hokuyo, state_commands )
{
    hok::state_command start( "BM" ); // starts transmission
    start.message_id.seq_num = 11;
    EXPECT_EQ( "BM;BM00000011\n\n", std::string( start.data(), hok::state_command::size ) );
    
    const char* reply = "BM;BM00000011\n00P\n\n";
    const hok::state_reply* state = reinterpret_cast< const hok::state_reply* >( reply );
    EXPECT_EQ( "BM00000011", state->message_id.str() );
    EXPECT_EQ( "00", std::string( state->status.data(), 2 ) );
    EXPECT_EQ( 0, state->status() );
    
}

