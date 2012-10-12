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

#ifndef SNARK_SENSORS_VELODYNE_THIN_THIN_H_
#define SNARK_SENSORS_VELODYNE_THIN_THIN_H_

#include <stdlib.h>
#include <snark/sensors/velodyne/db.h>
#include <snark/sensors/velodyne/packet.h>
#include <snark/sensors/velodyne/impl/get_laser_return.h>
#include <snark/sensors/velodyne/thin/focus.h>

namespace snark {  namespace velodyne { namespace thin {

/// thin packet by rate * 100%
void thin( velodyne::packet& packet, float rate );

/// thin packet, using given source of random numbers
template < typename Random >
void thin( velodyne::packet& packet, float rate, Random& random );

/// thin packet, using given source of random numbers
template < typename Random >
void thin( velodyne::packet& packet, const focus& focus, const db& db, Random& random );

/// write packet to thin buffer
std::size_t serialize( const velodyne::packet& packet, char* buf );

/// refill packet from thin buffer
velodyne::packet deserialize( const char* buf );

/// refill given packet from thin buffer
void deserialize( velodyne::packet& packet, const char* buf );

/// max block buffer size
enum { maxBlockBufferSize = 64 * 2 + 2 + 64 / 8 + 1 };

/// max buffer size
enum { maxBufferSize = 12 / 2 * maxBlockBufferSize + 1 };

template < typename Random >
void thin( velodyne::packet& packet, float rate, Random& random )
{
    for( unsigned int block = 0; block < packet.blocks.size(); ++block )
    {
        for( unsigned int laser = 0; laser < packet.blocks[block].lasers.size(); ++laser )
        {
            if( random() > rate ) { packet.blocks[block].lasers[laser].range = 0; }
        }
    }
}

template < typename Random >
void thin( velodyne::packet& packet, const focus& focus, const velodyne::db& db, double angularSpeed, Random& random )
{
    bool upper = true;
    for( unsigned int block = 0; block < packet.blocks.size(); ++block, upper = !upper )
    {
        for( unsigned int laser = 0; laser < packet.blocks[block].lasers.size(); ++laser )
        {
            velodyne::laser_return r = impl::getlaser_return( packet, block, laser, boost::posix_time::not_a_date_time, angularSpeed );
            double azimuth = db.lasers[r.id].azimuth( r.azimuth );
            double range = db.lasers[r.id].range( r.range );
            if( !focus.has( range, azimuth, 0, random ) ) { packet.blocks[block].lasers[laser].range = 0; }
        }
    }
}

} } } // namespace snark {  namespace velodyne { namespace thin {

#endif /*SNARK_SENSORS_VELODYNE_THIN_THIN_H_*/
