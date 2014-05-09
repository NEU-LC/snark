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


#include <gtest/gtest.h>
#include <comma/csv/stream.h>
#include <comma/csv/ascii.h>
#include <comma/visiting/traits.h>
#include "../battery.h"
#include "../traits.h"

#include <comma/base/types.h>
#include <comma/visiting/apply.h>
#include <comma/name_value/ptree.h>
#include <comma/csv/stream.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include <boost/property_tree/json_parser.hpp>

namespace snark { namespace ocean {

template < typename T >
comma::csv::ascii< T >& ascii() { 
    static comma::csv::ascii< T > ascii_;
    return ascii_;
}

typedef hex_value_t< comma::uint16 > hex_uint16;

TEST(ocean, ocean_raw_hex_data)
{
    std::string source = "10,197C";
    
    data_t pair;
    ascii< data_t >().get( pair, source );
    
    EXPECT_EQ( 16, pair.address.value );
    EXPECT_EQ( 6524, pair.value.value );
    
    source = "B14,17,0026,18,19c8,19,3840,1a,0010,1b,302f,1C,00cc";
    
    hex_data_t< 6 > data;
    ascii< hex_data_t< 6 > >().get( data, source );
    
    
    EXPECT_EQ( 4, data.battery_id );
    EXPECT_EQ( 1, data.controller_id );
    
    std::string temp;
    ascii< hex_data_t< 6 > >().put( data, temp );
    
    
    EXPECT_EQ( "\"B14\",17,26,18,19c8,19,3840,1a,10,1b,302f,1c,cc", temp );
    
}

TEST( ocean, setting_hex_data )
{
    std::vector< std::string > inputs;
    inputs.push_back( "$B11,02,000A,01,0294,03,0080,08,0B94,09,4115,0A,FFEC,0B,FEC3%3D" );
    inputs.push_back( "$B11,0C,000A,0D,0060,0E,0062,0F,1952,10,1A45,11,4B67,12,0519%33" );
    inputs.push_back( "$B11,13,FFFF,14,0000,15,41A0,16,00E0,17,0022,18,19C8,19,3840%3C" );
    inputs.push_back( "$B11,1A,0010,1B,2F56,1C,00B2%35" );
    inputs.push_back( "$B12,10,1999,11,FFFF,12,0533,13,FFFF,14,0000,15,41A0,16,00E0%4B" );
    inputs.push_back( "$B12,17,003B,18,19C8,19,3840,1A,0010,1B,2F56,1C,009A%34" );
    inputs.push_back( "$B13,10,19A4,11,FFFF,12,03E0,13,FFFF,14,0000,15,41A0,16,00E0%4C" );
    inputs.push_back( "$B13,17,0035,18,19C8,19,3840,1A,0010,1B,2F56,1C,009E%46" );
    
    ocean::controller_t< 3 > controller; // controller with three batteries
    
    for( std::size_t i=0; i<inputs.size(); ++i )
    {
        std::string& line = inputs[i];
        line = line.substr( 1, line.find_first_of('%') - 1 );
        std::vector< std::string > v = comma::split( line, ',');
        
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
    boost::property_tree::ptree t;
    comma::to_ptree to_ptree( t );
    comma::visiting::apply( to_ptree ).to( controller );
    std::ostringstream oss;
    boost::property_tree::write_json( oss, t );    
 
    EXPECT_EQ("json", oss.str() );
}


} } // namespace snark { namespace ocean {

int main( int argc, char* argv[] )
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
