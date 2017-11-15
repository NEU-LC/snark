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

#ifndef SNARK_SENSORS_VELODYNE_THIN_THIN_H_
#define SNARK_SENSORS_VELODYNE_THIN_THIN_H_

#include <stdlib.h>
#include "../hdl64/db.h"
#include "../hdl64/packet.h"
#include "../impl/get_laser_return.h"
#include "focus.h"

namespace snark {  namespace velodyne { namespace thin {

/// thin packet by rate * 100%
void thin( velodyne::hdl64::packet& packet, float rate );

/// thin packet, using given source of random numbers
template < typename Random >
void thin( velodyne::hdl64::packet& packet, float rate, Random& random );

/// thin packet, using given source of random numbers
template < typename Random >
void thin( velodyne::hdl64::packet& packet, const focus& focus, const hdl64::db& db, Random& random );

/// write packet to thin buffer
std::size_t serialize( const velodyne::hdl64::packet& packet, char* buf, comma::uint32 scan );

/// refill packet from thin buffer
/// @return velodyne packet and scan id
std::pair< velodyne::hdl64::packet, comma::uint32 > deserialize( const char* buf );

/// refill given packet from thin buffer
/// @return scan id
comma::uint32 deserialize( velodyne::hdl64::packet& packet, const char* buf );

/// max block buffer size
enum { maxBlockBufferSize = 64 * 2 + 2 + 64 / 8 + 1 };

/// max buffer size
enum { maxBufferSize = 12 / 2 * maxBlockBufferSize + 1 };

template < typename Random >
void thin( velodyne::hdl64::packet& packet, float rate, Random& random )
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
void thin( velodyne::hdl64::packet& packet, const focus& focus, const velodyne::hdl64::db& db, double angularSpeed, Random& random )
{
    bool upper = true;
    for( unsigned int block = 0; block < packet.blocks.size(); ++block, upper = !upper )
    {
        for( unsigned int laser = 0; laser < packet.blocks[block].lasers.size(); ++laser )
        {
            velodyne::laser_return r = impl::get_laser_return( packet, block, laser, boost::posix_time::not_a_date_time, angularSpeed );
            double azimuth = db.lasers[r.id].azimuth( r.azimuth );
            double range = db.lasers[r.id].range( r.range );
            if( !focus.has( range, azimuth, db.lasers[r.id].elevation, random ) ) { packet.blocks[block].lasers[laser].range = 0; }
        }
    }
}

} } } // namespace snark {  namespace velodyne { namespace thin {

#endif /*SNARK_SENSORS_VELODYNE_THIN_THIN_H_*/
