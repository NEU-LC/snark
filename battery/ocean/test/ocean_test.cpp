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
//    This product includes software developed by the University of Sydney.
// 4. Neither the name of the University of Sydney nor the
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

#include <gtest/gtest.h>
#include <comma/csv/stream.h>
#include <comma/csv/ascii.h>
#include <comma/visiting/traits.h>
#include <comma/base/types.h>
#include <comma/visiting/apply.h>
#include <comma/name_value/ptree.h>
#include <comma/csv/stream.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include <boost/property_tree/json_parser.hpp>
#include "../battery.h"
#include "../traits.h"
#include "../io_query.h"
#include "../packed_data.h"


namespace snark { namespace ocean {

template < typename T >
comma::csv::ascii< T >& ascii() { 
    static comma::csv::ascii< T > ascii_;
    return ascii_;
}

typedef hex_value_t< comma::uint16 > hex_uint16;

TEST(ocean, ocean_test_strip )
{
    std::string source = "$B15,17,0026,18,19c8,19,3840,1a,0010,1b,302f,1C,00cc%11\r";
    ocean::battery_t::strip( source );
    EXPECT_EQ("$B15,17,0026,18,19c8,19,3840,1a,0010,1b,302f,1C,00cc", source);

    source = "$B15,17,0026,18,19c8,19,3840,1a,0010,1b,302f,1C,00cc";
    ocean::battery_t::strip( source );
    EXPECT_EQ( source, source);

}

TEST(ocean, ocean_packed_data_test )
{
    std::string source = "$B15,17,0026,18,19c8,19,3840,1a,0010,1b,302f,1C,00cc%11";
    
    const packed::packet< 6 >* data = reinterpret_cast< const packed::packet< 6 >* >( &source[0] );
    EXPECT_EQ( 5, data->type.battery_id() );
    EXPECT_EQ( 1, data->type.controller_id() );
    EXPECT_EQ( 23,      data->values[0].address() );
    EXPECT_EQ( 38,      data->values[0].value() );
    EXPECT_EQ( 24,      data->values[1].address() );
    EXPECT_EQ( 6600,    data->values[1].value() );
    EXPECT_EQ( 25,      data->values[2].address() );
    EXPECT_EQ( 14400,   data->values[2].value() );
    EXPECT_EQ( 17,   data->crc() );
}

TEST(ocean, ocean_raw_hex_data)
{
    std::string source = "10,197C";
    
    data_t pair;
    ascii< data_t >().get( pair, source );
    
    EXPECT_EQ( 16, pair.address.value );
    EXPECT_EQ( 6524, pair.value.value );
    
    source = "$B15,17,0026,18,19c8,19,3840,1a,0010,1b,302f,1C,00cc%11";
    
    ocean::battery_t::strip( source );

    hex_data_t< 6 > data;
    ascii< hex_data_t< 6 > >().get( data, source );
    
    
    EXPECT_EQ( 5, data.type.battery_id() );
    EXPECT_EQ( 1, data.type.controller_id() );
    
    std::string temp;
    ascii< hex_data_t< 6 > >().put( data, temp );
    
    
    EXPECT_EQ( "\"$B15\",17,26,18,19c8,19,3840,1a,10,1b,302f,1c,cc", temp );

    EXPECT_EQ( 23,      data.values[0].address() );
    EXPECT_EQ( 38,      data.values[0].value() );
    EXPECT_EQ( 24,      data.values[1].address() );
    EXPECT_EQ( 6600,    data.values[1].value() );
    EXPECT_EQ( 25,      data.values[2].address() );
    EXPECT_EQ( 14400,   data.values[2].value() );
    
    // boost::property_tree::ptree t;
    // comma::to_ptree to_ptree( t );
    // comma::visiting::apply( to_ptree ).to( data );
    // std::ostringstream oss;
    // boost::property_tree::write_json( oss, t );    
 
    // EXPECT_EQ("json", oss.str() );
}

TEST( ocean, setting_hex_data )
{
    std::vector< std::string > inputs;
    inputs.push_back( "$B11,02,000A,01,0294,03,0001,08,0B96,09,415E,0A,0000,0B,0000%37" );
    inputs.push_back( "$B11,0C,0005,0D,0064,0E,0078,0F,1EE6,10,1EE6,11,FFFF,12,FFFF%4C" );
    inputs.push_back( "$B11,13,FFFF,14,0000,15,41A0,16,40E0,17,000B,18,19C8,19,3840%4A" );
    inputs.push_back( "$B11,1A,0031,1B,40E9,1C,0478%42" );
    inputs.push_back( "$B12,02,000A,01,0294,03,0001,08,0B95,09,4169,0A,0000,0B,0000%48" );
    inputs.push_back( "$B12,0C,0001,0D,0064,0E,0062,0F,190D,10,190D,11,FFFF,12,FFFF%40" );
    inputs.push_back( "$B12,13,FFFF,14,0000,15,41A0,16,40E0,17,000B,18,19C8,19,3840%49" );
    inputs.push_back( "$B12,1A,0031,1B,4350,1C,26D9%49" );
    inputs.push_back( "$B13,02,000A,01,0294,03,0001,08,0B99,09,407A,0A,0000,0B,0000%3D" );
    inputs.push_back( "$B13,0C,0001,0D,0064,0E,0061,0F,18C3,10,18E7,11,FFFF,12,FFFF%40" );
    inputs.push_back( "$B13,13,FFFF,14,0000,15,41A0,16,40E0,17,000A,18,19C8,19,3840%4B" );
    inputs.push_back( "$B13,1A,0031,1B,4350,1C,26BD%33" );

    
    ocean::controller< 3 > controller( 1 ); // controller with three batteries
    
    EXPECT_EQ( 10, address::current );

    for( std::size_t i=0; i<inputs.size(); ++i )
    {
        std::string& line = inputs[i];
        std::vector< std::string > v = comma::split( ocean::battery_t::strip( line ), ',');
        
        switch( v.size() )
        {
            case 15u: // 7 address & value pairs, 7*2 + 1 = 15
            {
                hex_data_t< 7 > data;
                ascii< hex_data_t< 7 > >().get( data, v );
                controller & data;
                break;
            }
            case 13u:
            {
                hex_data_t< 5 > data;
                ascii< hex_data_t< 5 > >().get( data, v );
                controller & data;
                break;
            }
            case 7u:
            {
                hex_data_t< 3 > data;
                ascii< hex_data_t< 3 > >().get( data, v );
                controller & data;
                break;
            }
            default:
            {
                EXPECT_TRUE( false );
                break;
            }
        }
    }

    controller.consolidate();

    std::string line;
    ascii< ocean::controller< 3 > >().put( controller, line );
    const std::string expected = "1,\"CH\",206.62,0,16.6617,100,1,\"CH\",16.734,0,0,296.6,79.1,100,1092.25,2,\"CH\",16.745,0,0,296.5,64.13,100,1092.25,3,\"CH\",16.506,0,0,296.9,63.39,100,1092.25";
    EXPECT_EQ( expected, line );

    // boost::property_tree::ptree t;
    // comma::to_ptree to_ptree( t );
    // comma::visiting::apply( to_ptree ).to( controller );
    // std::ostringstream oss;
    // boost::property_tree::write_json( oss, t );    
    // std::cerr << oss.str() << std::endl;
}

namespace impl_ {
    
class stdio_stub 
{
public:
    ocean8 read()
    {
        //char c;
        //std::cin.read( &c, 1u );
        //return c;
        
        if( last_written != 0 ) // return first byte
        {
            return ocean8( value & 0xff );
        }
        else    // return second byte
        {
            // return MSB byte
            return ocean8( (value & 0xff00) >> 8 );
        }
    }
    
    void write( ocean8 value )
    {
        //std::cout.write( (const char*) &value, 1u );
        last_written = value;
        if ( value != 0 ) { address = value & BOOST_BINARY( 11111 ); }
        else { return; }
        
        switch( address )
        {
            case ocean::address::current:
                this->value = 0x00ff;
                break;
            case ocean::address::average_current:
                this->value = 0x00ff;
                break;
            case ocean::address::temperature:
                this->value = comma::uint16( 0x0B96 );
                break;
            case ocean::address::voltage:
                this->value = comma::uint16( 0x415E );
                break;
            default:
                this->value = 0;
                break;
        }
    };
    
private:
    ocean8 address;
    comma::uint16 value; // value to write out
    ocean8 last_written; // last written value, eighter 'addrees' or 0
};

} //namespace impl_ {

TEST(ocean, ocean_binary_query )
{
    impl_::stdio_stub io;
    data_t current = query< 4, ocean::address::current >( io );
    EXPECT_EQ( 0xff, current.value() );
    EXPECT_EQ( 0x0a, current.address() );
    
    data_t temperature = query< 4, ocean::address::temperature >( io );
    EXPECT_EQ( 0x0B96, temperature.value() );
    EXPECT_EQ( 0x08, temperature.address() );
    
    controller< 4 > controller;
    query< impl_::stdio_stub >( controller, io );
    
    // boost::property_tree::ptree t;
    // comma::to_ptree to_ptree( t );
    // comma::visiting::apply( to_ptree ).to( controller );
    // std::ostringstream oss;
    // boost::property_tree::write_json( oss, t );    
    // std::cerr << oss.str() << std::endl;
}


} } // namespace snark { namespace ocean {

int main( int argc, char* argv[] )
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
