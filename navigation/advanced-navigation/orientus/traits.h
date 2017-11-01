// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
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

#pragma once

#include <string>
#include <comma/visiting/traits.h>
#include <comma/packed/traits.h>
#include "../../../math/spherical_geometry/traits.h"
#include "../../../timing/timestamped.h"
#include "../messages.h"

using namespace snark::navigation::advanced_navigation;

namespace comma { namespace visiting {

template < unsigned int Size, bool Signed, bool Floating, std::size_t N >
struct traits< boost::array< comma::packed::detail::little_endian< Size, Signed, Floating >, N > >
{
    template< typename K, typename V > static void visit( const K& k, const boost::array< comma::packed::detail::little_endian< Size, Signed, Floating >, N >& t, V& v )
    {
        for( std::size_t i = 0; i < t.size(); ++i ) { v.apply( std::string( 1, 'x' + i ).c_str(), t[i]() ); } // x, y, z
    }
};

template < typename T > struct traits< snark::timestamped< T > >
{
    template< typename K, typename V > static void visit( const K&, snark::timestamped< T >& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "", p.data );
    }

    template< typename K, typename V > static void visit( const K&, const snark::timestamped< T >& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "", p.data );
    }
};

template <>
struct traits< messages::header >
{
    template < typename Key, class Visitor > static void visit( const Key&, const messages::header& p, Visitor& v )
    {
        v.apply( "lrc", p.LRC() );
        v.apply( "id", p.id() );
        v.apply( "length", p.length() );
        v.apply( "crc", p.msg_crc() );
        v.apply( "valid", p.is_valid() );
    }
};

template <>
struct traits< messages::system_state >
{
    template < typename Key, class Visitor > static void visit( const Key&, const messages::system_state& p, Visitor& v )
    {
        v.apply( "system_status", p.system_status() );
        v.apply( "filter_status", p.filter_status() );
        // orientus outputs running time starting from 20150101T000000 UTC
        double running_time = ( p.unix_time_seconds() - 1420070400 ) + ( p.microseconds() * 1e-6 );
        v.apply( "running_time", running_time );
        v.apply( "orientation", p.orientation );
        v.apply( "angular_velocity", p.angular_velocity );
    }
};

template <>
struct traits< messages::raw_sensors >
{
    template < typename Key, class Visitor > static void visit( const Key&, const messages::raw_sensors& p, Visitor& v )
    {
        v.apply( "accelerometer", p.accelerometer );
        v.apply( "gyroscope", p.gyroscope );
        v.apply( "magnetometer", p.magnetometer );
        v.apply( "imu_temperature", p.imu_temperature() );
    }
};

} } // namespace comma { namespace visiting {
