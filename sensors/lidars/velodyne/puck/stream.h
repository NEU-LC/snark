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

#pragma once

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/math/compare.h>
#include "../laser_return.h" // quick and dirty
#include "../scan_tick.h" // todo: quick and dirty
#include "../impl/stream_traits.h"
#include "packet.h"

namespace snark {  namespace velodyne { namespace puck {

/// velodyne puck point stream
template < typename S >
class stream : public boost::noncopyable
{
    public:
        /// constructor
        stream( S* stream );

        /// read point, return NULL, if end of stream
        laser_return* read();

        /// skip given number of scans including the current one
        /// @todo: the same for packets and points, once needed
        void skip_scan();

        /// return current scan number
        unsigned int scan() const;

        /// interrupt reading
        void close();

    private:
        boost::scoped_ptr< S > stream_;
        boost::posix_time::ptime timestamp_;
        const char* buffer_;
        const packet* packet_;
        packet::const_iterator puck_packet_iterator_;
        unsigned int scan_;
        scan_tick tick_;
        bool closed_;
        laser_return laser_return_;
};


template < typename S >
inline stream< S >::stream( S* stream ) : stream_( stream ), buffer_( NULL ), scan_( 0 ), closed_( false ) {}

template < typename S >
inline laser_return* stream< S >::read()
{
    while( !closed_ )
    {
        if( !buffer_ )
        {
            buffer_ = impl::stream_traits< S >::read( *stream_, sizeof( packet ) );
            if( !buffer_ ) { closed_ = true; return NULL; }
            puck_packet_iterator_ = puck::packet::const_iterator( reinterpret_cast< const puck::packet* >( buffer_ ) );
            timestamp_ = impl::stream_traits< S >::timestamp( *stream_ );
        }
        if( timestamp_.is_not_a_date_time() ) { timestamp_ = impl::stream_traits< S >::timestamp( *stream_ ); }
        const puck::packet::const_iterator::value_type& v = *puck_packet_iterator_;
        laser_return_.id = v.id; // todo? use laser_return type in puck?
        laser_return_.azimuth = v.azimuth;
        laser_return_.intensity = v.reflectivity;
        laser_return_.range = v.range;
        laser_return_.timestamp = timestamp_ + boost::posix_time::microseconds( v.delay );
        ++puck_packet_iterator_;
        if( puck_packet_iterator_.done() ) { buffer_ = NULL; }
        bool valid = !comma::math::equal( laser_return_.range, 0 );
        return &laser_return_;
    }
    return NULL;
}

template < typename S >
inline unsigned int stream< S >::scan() const { return scan_; }

template < typename S >
inline void stream< S >::close() { m_closed = true; impl::stream_traits< S >::close( *stream_ ); }

template < typename S >
inline void stream< S >::skip_scan()
{
    while( !closed_ )
    {
        puck_packet_iterator_ = puck::packet::const_iterator( reinterpret_cast< const puck::packet* >( buffer_ ) );
        packet_ = reinterpret_cast< const packet* >( impl::stream_traits< S >::read( *stream_, sizeof( packet ) ) );
        if( packet_ == NULL ) { return; }
        if( tick_.is_new_scan( *packet_ ) ) { ++scan_; return; }
        if( impl::stream_traits< S >::is_new_scan( tick_, *stream_, *packet_ ) ) { ++scan_; return; }
    }
}

} } } // namespace snark {  namespace velodyne { namespace puck {
