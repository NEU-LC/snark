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

/// @author vsevolod vlaskine

#ifndef SNARK_NAVIGATION_NMEA_TRAITS_H_
#define SNARK_NAVIGATION_NMEA_TRAITS_H_

#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/visiting/traits.h>
#include "message.h"

namespace comma { namespace visiting {

template <> struct traits< snark::nmea::messages::coordinate >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::nmea::messages::coordinate& p, Visitor& v ) // hyper-quick and dirty
    {
        double c;
        char d;
        v.apply( "value", c );
        v.apply( "direction", d );
        double sign = d == 'N' || d == 'E' ? 1 : -1;
        int degrees = c / 100;
        double fractions = ( c - degrees ) / 60 ;
        p.value = sign * ( M_PI * ( fractions + degrees ) ) / 180;
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::nmea::messages::coordinate& p, Visitor& v ) // hyper-quick and dirty
    {
        double c = 0; // todo, if needed: set from p
        char d = 0; // todo, if needed: set from p
        v.apply( "value", c );
        v.apply( "direction", d );
    }
};

template <> struct traits< snark::nmea::messages::time > // pain
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::nmea::messages::time& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        std::string t;
        v.apply( "time_of_day", t );
        p.value = boost::posix_time::ptime( boost::posix_time::microsec_clock::universal_time().date(), boost::posix_time::from_iso_string( "19700101T" + t ).time_of_day() );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::nmea::messages::time& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        std::string t; // todo, if needed: set from p
        v.apply( "time_of_day", t );
    }
};

template < typename T > struct traits< snark::nmea::message< T > >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::nmea::message< T >& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        v.apply( "type", p.type );
        v.apply( "value", p.value );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::nmea::message< T >& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        v.apply( "type", p.type );
        v.apply( "value", p.value );
    }
};

template < typename T > struct traits< snark::nmea::messages::ptnl::message< T > >
{
    template < typename Key, class Visitor >
    static void visit( const Key& k, snark::nmea::messages::ptnl::message< T >& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        visit( k, static_cast< snark::nmea::message< T >& >( p ), v );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key& k, const snark::nmea::messages::ptnl::message< T >& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        visit( k, static_cast< const snark::nmea::message< T >& >( p ), v );
    }
};

template <> struct traits< snark::nmea::messages::gpgga >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::nmea::messages::gpgga& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        v.apply( "time", p.time );
        v.apply( "coordinates", p.coordinates );
        unsigned int q;
        v.apply( "quality", q );
        p.quality = static_cast< snark::nmea::messages::gpgga::quality_t >( q );
        v.apply( "satellites_in_use", p.satellites_in_use );
        v.apply( "hdop", p.hdop );
        v.apply( "orthometric_height", p.orthometric_height );
        v.apply( "height_unit", p.height_unit );
        v.apply( "geoid_separation", p.geoid_separation );
        v.apply( "geoid_separation_unit", p.geoid_separation_unit );
        v.apply( "age_of_differential_gps_data_record", p.age_of_differential_gps_data_record );
        v.apply( "reference_station_id", p.reference_station_id );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::nmea::messages::gpgga& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        v.apply( "time", p.time );
        v.apply( "coordinates", p.coordinates );
        v.apply( "quality", static_cast< unsigned int >( p.quality ) );        
        v.apply( "satellites_in_use", p.satellites_in_use );
        v.apply( "hdop", p.hdop );
        v.apply( "orthometric_height", p.orthometric_height );
        v.apply( "height_unit", p.height_unit );
        v.apply( "geoid_separation", p.geoid_separation );
        v.apply( "geoid_separation_unit", p.geoid_separation_unit );
        v.apply( "age_of_differential_gps_data_record", p.age_of_differential_gps_data_record );
        v.apply( "reference_station_id", p.reference_station_id );
    }
};

template <> struct traits< snark::nmea::messages::ptnl::avr >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::nmea::messages::ptnl::avr& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        v.apply( "time", p.time );
        v.apply( "yaw", p.yaw );
        v.apply( "yaw_string", p.yaw_string );
        v.apply( "tilt", p.tilt );
        v.apply( "tilt_string", p.tilt_string );
        v.apply( "roll", p.roll );
        v.apply( "roll_string", p.roll_string );
        v.apply( "range", p.range );
        v.apply( "quality", static_cast< unsigned int >( p.quality ) );        
        v.apply( "pdop", p.pdop );
        v.apply( "satellites_in_use", p.satellites_in_use );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_NAVIGATION_NMEA_TRAITS_H_
