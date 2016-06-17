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

#include <cmath>
#include "packet.h"

namespace snark { namespace velodyne { namespace puck {

namespace timing {

static double firing_interval = ( 2.304 / 1000000 );

static double recharge_interval = ( 18.43 / 1000000 );

} // namespace timing {

static double azimuth_step( double from, double to )
{
    static double ratio = timing::firing_interval / ( timing::firing_interval + timing::recharge_interval / packet::number_of_lasers );
    double diff = to - from;
    if( diff < 0 ) { diff += M_PI * 2; }
    return diff * ratio;
}

packet::const_iterator::const_iterator() : packet_( NULL ), done_( true ) {}

packet::const_iterator::const_iterator( const packet* p )
    : packet_( p )
    , block_( 0 )
    , subblock_( 0 )
    , done_( false )
{
    value_.azimuth = packet_->blocks[0].azimuth_as_radians();
    azimuth_step_ = azimuth_step( value_.azimuth, packet_->blocks[1].azimuth_as_radians() );
}

packet::const_iterator::value_type packet::const_iterator::operator->() const { return operator*(); }

packet::const_iterator::value_type packet::const_iterator::operator*() const
{
    packet::const_iterator::value_type v;
    const packet::laser_return& r = packet_->blocks[block_].channels[subblock_][value_.id];
    v.range = r.range_as_meters();
    v.reflectivity = r.reflectivity();
    return v;
}

void packet::const_iterator::operator++()
{
    if( packet_->factory.mode() == packet::factory_t::modes::dual_return )
    {
        ++subblock_;
        if( subblock_ < packet::number_of_subblocks ) { return; }
        subblock_ = 0;
        ++value_.id;
        if( value_.id < packet::number_of_lasers ) { return; }
        value_.id = 0;
        value_.delay += timing::firing_interval;
    }
    else
    {
        ++value_.id;
        if( value_.id < packet::number_of_lasers ) { value_.delay += timing::firing_interval; return; }
        value_.id = 0;
        ++subblock_;
        if( subblock_ < packet::number_of_subblocks ) { value_.delay += timing::recharge_interval; return; }
        subblock_ = 0;
    }
    ++block_;
    value_.delay += timing::recharge_interval;
    if( block_ == packet::number_of_blocks ) { done_ = true; return; }
    value_.azimuth = packet_->blocks[block_].azimuth_as_radians();
    if( block_ < ( packet::number_of_blocks - 1 ) ) { azimuth_step_ = azimuth_step( value_.azimuth, packet_->blocks[ block_ + 1 ].azimuth_as_radians() ); }
}

bool packet::const_iterator::done() { return done_; }

} } } // namespace snark { namespace velodyne { namespace puck {
