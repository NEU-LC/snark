// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2018 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

// snark is a generic and flexible library for robotics research
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

#pragma once

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/math/compare.h>
#include <comma/csv/impl/epoch.h>
#include "../../laser_return.h" // quick and dirty
#include "../../scan_tick.h" // todo: quick and dirty
#include "../../impl/stream_traits.h"
#include "../../stream.h"
#include "packet.h"
#include "ntp.h"
#include "../../packet_traits.h"

namespace snark { namespace robosense {

/// point stream
template < typename S >
class stream : public boost::noncopyable, public velodyne::stream
{
    public:
        /// constructor
        stream( S* stream, bool output_invalid = false, const boost::optional< ntp_t >& ntp = boost::none );

        /// read point, return NULL, if end of stream
        velodyne::laser_return* read();

        /// skip given number of scans including the current one
        /// @todo: the same for packets and points, once needed
        void skip_scan();

        /// return current scan number
        unsigned int scan() const;

        /// interrupt reading
        void close();

        /// return true if scan is valid
        bool is_scan_valid() const;
        
        unsigned int packet_duration() const { return velodyne::packet_traits< msop::packet >::packet_duration; } // todo: confirm packet duration (currently assumed 100 microseconds)
        
    private:
        boost::scoped_ptr< S > stream_;
        boost::posix_time::ptime timestamp_;
        const char* buffer_;
        msop::packet::const_iterator packet_iterator_;
        unsigned int scan_;
        bool closed_;
        velodyne::laser_return laser_return_;
        bool output_invalid_;
        boost::optional<ntp_t> ntp_;
        bool is_scan_valid_;
};


template < typename S >
inline stream< S >::stream( S* stream, bool output_invalid, const boost::optional< ntp_t >& ntp ) : stream_( stream ), buffer_( NULL ), scan_( 0 ), closed_( false ), output_invalid_( output_invalid ), ntp_(ntp), is_scan_valid_(true) {}

template < typename S >
inline velodyne::laser_return* stream< S >::read()
{
    while( !closed_ )
    {
        if( !buffer_ )
        {
            buffer_ = velodyne::impl::stream_traits< S >::read( *stream_, msop::packet::size );
            if( !buffer_ ) { closed_ = true; return NULL; }
            const msop::packet* p = reinterpret_cast< const msop::packet* >( buffer_ );
            auto res = velodyne::impl::stream_traits< S >::is_new_scan( scan_tick_, *stream_, *p );
            is_scan_valid_ = res.second;
            if( res.first ) { ++scan_; }
            packet_iterator_ = msop::packet::const_iterator( p );
            timestamp_ = velodyne::impl::stream_traits< S >::timestamp( *stream_ ); //timestamp_ = ntp_ ? ntp_->update_timestamp( velodyne::impl::stream_traits< S >::timestamp( *stream_ ), p->timestamp() ) : velodyne::impl::stream_traits< S >::timestamp( *stream_ );
        }
        if( timestamp_.is_not_a_date_time() ) { timestamp_ = velodyne::impl::stream_traits< S >::timestamp( *stream_ ); }
        const msop::packet::const_iterator::value_type& v = *packet_iterator_;
        laser_return_.id = v.id; // todo? reuse laser_return type in puck?
        laser_return_.azimuth = v.azimuth;
        laser_return_.intensity = v.reflectivity;
        laser_return_.range = v.range;
        laser_return_.timestamp = timestamp_ + boost::posix_time::microseconds( v.delay );
        ++packet_iterator_;
        if( packet_iterator_.done() ) { buffer_ = NULL; }
        bool valid = !comma::math::equal( laser_return_.range, 0 );
        if( valid || output_invalid_ ) { return &laser_return_; }
    }
    return NULL;
}

template < typename S >
inline unsigned int stream< S >::scan() const { return scan_; }

template < typename S >
inline void stream< S >::close() { closed_ = true; velodyne::impl::stream_traits< S >::close( *stream_ ); }

template < typename S >
inline bool stream< S >::is_scan_valid() const { return is_scan_valid_; }

template < typename S >
inline void stream< S >::skip_scan()
{
    while( !closed_ )
    {
        const msop::packet* p = reinterpret_cast< const msop::packet* >( velodyne::impl::stream_traits< S >::read( *stream_, msop::packet::size ) );
        if( p == NULL ) { return; }
        if( scan_tick_.is_new_scan( *p, velodyne::impl::stream_traits< S >::timestamp( *stream_ ) ).first ) { ++scan_; return; }
        if( velodyne::impl::stream_traits< S >::is_new_scan( scan_tick_, *stream_, *p ).first ) { ++scan_; return; }
    }
}

} } // namespace snark { namespace robosense {
