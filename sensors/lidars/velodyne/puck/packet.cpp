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

#include "packet.h"

namespace snark { namespace velodyne { namespace puck {

packet::const_iterator::const_iterator() : packet_( NULL ), done_( true ) {}

packet::const_iterator::const_iterator( const packet* p )
    : packet_( p )
    , block_( 0 )
    , channel_( std::make_pair( 0, 0 ) )
    , azimuth_( packet_->blocks[0].azimuth_as_radians() )
    , done_( false )
{
    azimuth_step_ = 0; // todo
}

packet::const_iterator::value_type packet::const_iterator::operator->() const { return operator*(); }
        
packet::const_iterator::value_type packet::const_iterator::operator*() const
{
    packet::const_iterator::value_type v;
    v.id = channel_.second;
    const packet::laser_return& r = packet_->blocks[block_].channels[channel_.first][channel_.second];
    v.azimuth = azimuth_;
    v.range = r.range_as_meters();
    v.reflectivity = r.reflectivity();
    return v;
}

void packet::const_iterator::operator++()
{
    ++channel_.second;
    if( channel_.second < packet::number_of_lasers ) { return; }
    channel_.second = 0;
    ++channel_.first;
    if( channel_.first < 2 ) { return; }
    channel_.first = 0;
    ++block_;
    if( block_ < packet::number_of_blocks ) { return; }
    done_ = true;
}

bool packet::const_iterator::done() { return done_; }

} } } // namespace snark { namespace velodyne { namespace puck {
