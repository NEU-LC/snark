// Copyright (c) 2017 The University of Sydney

#pragma once

#include "../messages.h"
#include "../../../math/spherical_geometry/traits.h"
#include "../../../timing/timestamped.h"
#include <comma/visiting/traits.h>
#include <string>

namespace comma { namespace visiting {

template < unsigned int Size, bool Signed, bool Floating, std::size_t N >
struct traits< boost::array< comma::packed::detail::endian< comma::packed::detail::little, Size, Signed, Floating >, N > >
{
    template< typename K, typename V > static void visit( const K& k, const boost::array< comma::packed::detail::endian< comma::packed::detail::little, Size, Signed, Floating >, N >& t, V& v )
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
struct traits< snark::navigation::advanced_navigation::messages::header >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::header& p, Visitor& v )
    {
        v.apply( "lrc", p.LRC() );
        v.apply( "id", p.id() );
        v.apply( "length", p.len() );
        v.apply( "crc", p.msg_crc() );
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::system_state >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::system_state& p, Visitor& v )
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
struct traits< snark::navigation::advanced_navigation::messages::raw_sensors >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::raw_sensors& p, Visitor& v )
    {
        v.apply( "accelerometer", p.accelerometer );
        v.apply( "gyroscope", p.gyroscope );
        v.apply( "magnetometer", p.magnetometer );
        v.apply( "imu_temperature", p.imu_temperature() );
    }
};

} } // namespace comma { namespace visiting {
