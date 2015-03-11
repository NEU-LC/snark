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

#ifndef SNARK_OCEAN_HEX_VALUE
#define SNARK_OCEAN_HEX_VALUE
#include <comma/base/types.h>
#include <comma/packed/packed.h>
#include <boost/array.hpp>
#include <vector>
#include <iomanip>
#include <iostream>
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include "packed_data.h"

namespace snark { namespace ocean {
    
typedef unsigned char uint8;

template < typename T > struct signed_type;
template < > struct signed_type< comma::uint16 >
{
    typedef comma::int16 type;
};


/// Wraps a fundamental type so that is can be serialise/deserialised to/from hex CSV string
template < typename T >
struct hex_value_t
{
    hex_value_t() : value(0)
    {
        BOOST_STATIC_ASSERT_MSG( boost::is_integral< T >::value, " T must be integral type" );
        BOOST_STATIC_ASSERT_MSG( (!boost::is_same< T, unsigned char >::value), " T must not be unsigned char, use unsigned short instead" );
        BOOST_STATIC_ASSERT_MSG( (!boost::is_same< T, char >::value), " T must not be char, use short instead" );

    }
    hex_value_t( T v ) : value(v)
    {

    }
    comma::uint16 byte() const  { return comma::uint16( value & 0x000000ff ); }
    typename signed_type< T >::type cast() const { return typename signed_type< T >::type(value); }
    
    T operator()() const { return value; }

    
    T value;
};

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

/// A pair of data ( 1 byte) address and value ( 2 bytes )
/// See universal Smart Battery Specification doc
struct data_t
{
    data_t() : address( 0 ), value( 0 ) {}
    data_t( comma::uint16 addr, comma::uint16 val ) : address( addr ), value( val ) {}
    hex_value_t< comma::uint16 > address;
    hex_value_t< comma::uint16 > value;
};

/// Store values pushed from the Ocean controller
/// N is the number of data value pairs
template < int N >
struct hex_data_t
{
///    hex_data_t() : controller_id(0), battery_id(0) {} 
    
    static const char battery_char = 'B';
    static const char setup_char = 'S';
    ocean::packed::type_info type;
    /// Pairs of address identifying data and raw value
    boost::array< data_t, N > values;
    typedef typename boost::array< data_t, N >::iterator value_iter;
    
    bool is_battery_data() const { return type.category()[0] == battery_char; }
};

} } // namespace snark { namespace ocean {

#endif //  SNARK_OCEAN_HEX_VALUE
