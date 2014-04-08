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

namespace snark { namespace ocean {

template < typename T >
comma::csv::ascii< T >& ascii() { 
    static comma::csv::ascii< T > ascii_;
    return ascii_;
}

typedef hex_value_t< comma::uint16 > hex_uint16;
/// For lexical_cast
template < typename T >
std::ostream& operator<<( std::ostream& ostream, const hex_value_t< T >& val )
{
    ostream << std::hex << val.value;
    return ostream;
}

/// For lexical_cast
template < typename T >
std::istream& operator>>( std::istream& istream, hex_value_t< T >& val )
{
    istream >> std::hex >> val.value;
    return istream;
}

TEST(time, ocean_raw_hex_data)
{
    std::string source = "10,197C";
    
    data_t pair;
    ascii< data_t >().get( pair, source );
    
    EXPECT_EQ( 16, pair.address.value );
    EXPECT_EQ( 6524, pair.value.value );
    
    source = "B14,17,0026,18,19c8,19,3840,1a,0010,1b,302f,1C,00cc";
    
    hex_data_t data;
    ascii< hex_data_t >().get( data, source );
    
    std::string temp;
    ascii< hex_data_t >().put( data, temp );
    
    
    EXPECT_EQ( "\"B14\",17,26,18,19c8,19,3840,1a,10,1b,302f,1c,cc", temp );
    
}


} } // namespace snark { namespace ocean {

int main( int argc, char* argv[] )
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
