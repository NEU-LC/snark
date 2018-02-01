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

#include <stdlib.h>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/math/compare.h>
#include "db.h"
#include "../laser_return.h"
#include "../scan_tick.h"
#include "../stream.h"
#include "../impl/stream_traits.h"
#include <comma/application/verbose.h>
#include "../packet_traits.h"

namespace snark { namespace velodyne { namespace hdl64 {

/// velodyne point stream
template < typename S >
class stream : public velodyne::stream, public boost::noncopyable
{
    public:
        /// constructor
        stream( S* stream, unsigned int rpm, bool outputInvalid = false, bool legacy = false );

        /// constructor
        stream( S* stream, bool outputInvalid = false, bool legacy = false );

        /// read point, return NULL, if end of stream
        laser_return* read();

        /// skip given number of scans including the current one
        /// @todo: the same for packets and points, once needed
        void skip_scan();

        /// return current scan number
        unsigned int scan() const;

        /// interrupt reading
        void close();
        
        /// return true if scan is valid
        bool is_scan_valid();
        
        unsigned packet_duration() { return packet_traits<packet>::packet_duration; }
        
    private:
        boost::optional< double > m_angularSpeed;
        bool m_outputInvalid;
        boost::scoped_ptr< S > m_stream;
        boost::posix_time::ptime m_timestamp;
        const char* buffer_;
        const packet* m_packet;
        enum { m_size = packet::number_of_blocks * packet::returns_per_block };
        struct index // quick and dirty
        {
            unsigned int idx;
            unsigned int block;
            unsigned int laser;
            index() : idx( 0 ), block( 0 ), laser( 0 ) {}
            const index& operator++()
            {
                ++idx;
                if( block & 0x1 )
                {
                    ++laser;
                    if( laser < packet::returns_per_block ) { --block; } else { laser = 0; ++block; }
                }
                else
                {
                    ++block;
                }
                return *this;
            }
            bool operator==( const index& rhs ) const { return idx == rhs.idx; }
        };
        index m_index;
        unsigned int m_scan;
        bool m_closed;
        laser_return m_laserReturn;
        double angularSpeed();
        bool m_legacy;
        bool is_scan_valid_;
};

template < typename S >
inline stream< S >::stream( S* stream, unsigned int rpm, bool outputInvalid, bool legacy )
    : m_angularSpeed( ( 360 / 60 ) * rpm )
    , m_outputInvalid( outputInvalid )
    , m_stream( stream )
    , buffer_( NULL )
    , m_scan( 0 )
    , m_closed( false )
    , m_legacy(legacy)
    , is_scan_valid_(true)
{
    m_index.idx = m_size;
}

template < typename S >
inline stream< S >::stream( S* stream, bool outputInvalid, bool legacy )
    : m_outputInvalid( outputInvalid )
    , m_stream( stream )
    , buffer_( NULL )
    , m_scan( 0 )
    , m_closed( false )
    , m_legacy(legacy)
    , is_scan_valid_(true)
{
    m_index.idx = m_size;
}

template < typename S >
inline double stream< S >::angularSpeed()
{
    if( m_angularSpeed ) { return *m_angularSpeed; }
    double da = double( m_packet->blocks[0].rotation() - m_packet->blocks[ packet::number_of_blocks - 1 ].rotation() ) / 100;
    double dt = impl::time_span(m_legacy);
    return da / dt;
}

template < typename S >
inline laser_return* stream< S >::read()
{
    while( !m_closed )
    {
        if( !buffer_ )
        {
            buffer_ = impl::stream_traits< S >::read( *m_stream, sizeof( packet ) );
            if( !buffer_ ) { m_closed = true; return NULL; }
            m_packet = reinterpret_cast< const packet* >( buffer_ );
            auto res=impl::stream_traits< S >::is_new_scan( scan_tick_, *m_stream, *m_packet );
            is_scan_valid_=res.second;
            if( res.first ) { ++m_scan; 
//                 comma::verbose<<"new scan "<<m_scan<<", "<<is_scan_valid_<<std::endl;
                
            } //if( scan_tick_.is_new_scan( *m_packet ) ) { ++m_scan; }
            m_index = index();
            m_timestamp = impl::stream_traits< S >::timestamp( *m_stream );
        }
        if( m_timestamp.is_not_a_date_time() ) { m_timestamp = impl::stream_traits< S >::timestamp( *m_stream ); }
        m_laserReturn = impl::get_laser_return( *m_packet, m_index.block, m_index.laser, m_timestamp, angularSpeed(), m_legacy );
        ++m_index;
        if( m_index.idx >= m_size ) { buffer_ = NULL; }
        bool valid = !comma::math::equal( m_laserReturn.range, 0 );
        if( valid || m_outputInvalid ) { return &m_laserReturn; }
    }
    return NULL;
}

template < typename S >
inline unsigned int stream< S >::scan() const { return m_scan; }

// static int debug=0;
template < typename S >
inline bool stream< S >::is_scan_valid() 
{ 
//     if(debug<400000)
//     {
//         if(!(debug%20000))
//             comma::verbose<<"hdl64 stream is_scan_valid "<<is_scan_valid_<<std::endl;
//         debug++;
//     }
    return is_scan_valid_; 
    
}

template < typename S >
inline void stream< S >::close() { m_closed = true; impl::stream_traits< S >::close( *m_stream ); }

template < typename S >
inline void stream< S >::skip_scan()
{
    while( !m_closed )
    {
        m_index = index();
        m_packet = reinterpret_cast< const packet* >( impl::stream_traits< S >::read( *m_stream, sizeof( packet ) ) );
        if( m_packet == NULL ) { return; }
        if( scan_tick_.is_new_scan( *m_packet, impl::stream_traits< S >::timestamp( *m_stream ) ).first ) { ++m_scan; return; }
        if( impl::stream_traits< S >::is_new_scan( scan_tick_, *m_stream, *m_packet ).first ) { ++m_scan; return; }
    }
}

} } } // namespace snark {  namespace velodyne { namespace hdl64 {
