// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <comma/base/types.h>
#include <comma/base/exception.h>
#include <snark/sensors/velodyne/packet.h>
#include "./thin.h"

namespace snark {  namespace velodyne { namespace thin {

static boost::mt19937 generator;

void thin( velodyne::packet& packet, float rate )
{
    boost::uniform_real< float > distribution( 0, 1 ); // watch performance
    boost::variate_generator< boost::mt19937&, boost::uniform_real< float > > random( generator, distribution );
    thin( packet, rate, random );
}

static void set( char* ids, unsigned int i, bool value = true )
{
    if( value ) { *( ids + ( i / 8 ) ) |= ( 1 << ( i % 8 ) ); }
    else { *( ids + ( i / 8 ) ) &= ~( 1 << ( i % 8 ) ); }
}

static bool get( const char* ids, unsigned int i )
{
    return *( ids + ( i / 8 ) ) & ( 1 << ( i % 8 ) );
}

const unsigned int idsSize = 64 / 8;

static std::size_t serialize( const velodyne::packet::laser_block& upper, const velodyne::packet::laser_block& lower, char* buf )
{
    char* begin = buf;
    ::memcpy( buf, upper.rotation.data(), 2 );
    buf += 2;
    char* ids = buf;
    ::memset( ids, 0, idsSize );
    buf += idsSize;
    bool modified = false;
    for( unsigned int i = 0; i < upper.lasers.size(); ++i )
    {
        if( !upper.lasers[i].range() != 0 ) { continue; }
        set( ids, i );
        ::memcpy( buf, upper.lasers[i].range.data(), 2 );
        buf += 2;
        modified = true;
    }
    for( unsigned int i = 0; i < lower.lasers.size(); ++i )
    {
        if( !lower.lasers[i].range() != 0 ) { continue; }
        set( ids, i + lower.lasers.size() );
        ::memcpy( buf, lower.lasers[i].range.data(), 2 );
        buf += 2;
        modified = true;
    }
    return modified ? buf - begin : 0;
}

std::size_t serialize( const velodyne::packet& packet, char* buf ) // quick and dirty, dirties the buffer
{
    char* begin = buf;
    char& ids = *buf++;
    ids = 0x00;
    for( unsigned int i = 0; i < packet.blocks.size(); i += 2 )
    {
        unsigned int size = serialize( packet.blocks[ i ], packet.blocks[ i + 1 ], buf );
        if( size > 0 )
        {
            ids |= ( 1 << ( i >> 1 ) );
            buf += size;
        }
    }
    return buf - begin;
}

static std::size_t deserialize( velodyne::packet::laser_block& upper, velodyne::packet::laser_block& lower, const char* buf )
{
    upper.id = velodyne::packet::upper_block_id();
    lower.id = velodyne::packet::lower_block_id();
    const char* begin = buf;
    ::memcpy( upper.rotation.data(), buf, 2 );
    ::memcpy( lower.rotation.data(), buf, 2 );
    buf += 2;
    const char* ids = buf;
    buf += idsSize;
    for( unsigned int i = 0; i < upper.lasers.size(); ++i )
    {
        if( !get( ids, i ) ) { continue; }
        ::memcpy( upper.lasers[i].range.data(), buf, 2 );
        buf += 2;
    }
    for( unsigned int i = 0; i < lower.lasers.size(); ++i )
    {
        if( !get( ids, i + lower.lasers.size() ) ) { continue; }
        ::memcpy( lower.lasers[i].range.data(), buf, 2 );
        buf += 2;
    }
    return buf - begin;
}

void deserialize( velodyne::packet& packet, const char* buf )
{
    ::memset( &packet, 0, velodyne::packet::size );
    const char& ids = *buf++;
    for( unsigned int i = 0; i < packet.blocks.size(); i += 2 )
    {
        if( !( ids & ( 1 << ( i >> 1 ) ) ) ) { continue; }
        buf += deserialize( packet.blocks[i], packet.blocks[ i + 1 ], buf );
    }
}

velodyne::packet deserialize( const char* buf )
{
    velodyne::packet packet;
    deserialize( packet, buf );
    return packet;
}

} } } // namespace snark {  namespace velodyne { namespace thin {
