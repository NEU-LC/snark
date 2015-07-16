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

#ifndef SNARK_OCEAN_TRAITS_H
#define SNARK_OCEAN_TRAITS_H
#include <comma/visiting/visit.h>
#include <comma/string/string.h>
#include <boost/lexical_cast.hpp>
#include "hex_value.h"
#include "battery.h"

namespace boost {
    
/// Make hex_value_t a fundamental type so far as visiting is concerned
/// It is lexical_cast able from/to hex string
template < typename T > struct is_fundamental< snark::ocean::hex_value_t< T > > : public true_type {};

}

namespace comma { namespace visiting {
using snark::ocean::data_t;
using snark::ocean::hex_value_t;
using snark::ocean::hex_data_t;
using snark::ocean::uint8;
using snark::ocean::battery_t;
using snark::ocean::controller;


template <> struct traits< data_t >
{
    template< typename K, typename V > static void visit( const K& k, data_t& t, V& v )
    {
        v.apply("address", t.address);
        v.apply("value", t.value);
    }

    template< typename K, typename V > static void visit( const K& k, const data_t& t, V& v )
    {
        v.apply("address", t.address );
        v.apply("value", t.value );
    }
};

template < int N > struct traits< boost::array< int, N > >
{
    template< typename K, typename V > static void visit( const K& k, boost::array< int, N >& t, V& v )
    {
        for( std::size_t i=0; i<N; ++i ) { v.apply( boost::lexical_cast< std::string >( i ), t[i] ); }
    }

    template< typename K, typename V > static void visit( const K& k, const boost::array< int, N >& t, V& v )
    {
        for( std::size_t i=0; i<N; ++i ) { v.apply( boost::lexical_cast< std::string >( i ), t[i] ); }
    }
};
    
template < int N > struct traits< hex_data_t< N > >
{
    template< typename K, typename V > static void visit( const K& k, hex_data_t< N >& t, V& v )
    {
        using snark::ocean::packed::type_info;
        std::string type;
        v.apply("type", type );
        const type_info* info = reinterpret_cast< type_info* >( &type[0] );
        t.type = *info;
        v.apply("values", t.values);
    }

    template< typename K, typename V > static void visit( const K& k, const hex_data_t< N >& t, V& v )
    {
        v.apply("type", std::string( t.type.data(), t.type.size ) );
        v.apply("values", t.values);
    }
};

template <> struct traits< battery_t >
{
    template< typename K, typename V > static void visit( const K& k, const battery_t& t, V& v )
    {
        v.apply("id", int(t.id) );
        v.apply("state", battery_t::state_to_string(t.state) );
        v.apply("voltage", t.voltage.value() );
        v.apply("current", t.current.value() );
        v.apply("average_current", t.average_current.value() );
        v.apply("temperature", t.temperature.value() );
        v.apply("remaining_capacity", t.remaining_capacity.value() );
        v.apply("charge_pc", t.charge_pc );
        v.apply("time_to_empty", t.time_to_empty.total_seconds() / 3600.0 );
    }
};

template < int N > struct traits< controller< N > >
{
    template< typename K, typename V > static void visit( const K& k, const controller< N >& t, V& v )
    {
        v.apply("id", int(t.id) );
        v.apply("state", battery_t::state_to_string(t.state) );
        v.apply("total_power", t.total_power.value() );
        v.apply("total_current", t.total_current.value() );
        v.apply("average_voltage", t.average_voltage.value() );
        v.apply("average_charge", t.average_charge );
        v.apply("batteries", t.batteries );
    }
};   


} } // namespace comma { namespace visiting {

#endif // SNARK_OCEAN_TRAITS_H
