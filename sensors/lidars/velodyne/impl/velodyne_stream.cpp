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

#include "velodyne_stream.h"

namespace snark {

velodyne_stream::velodyne_stream( 
                      velodyne::stream* s
                    , velodyne::calculator* calculator
                    , boost::optional< std::size_t > from
                    , boost::optional< std::size_t > to 
                    , bool raw_intensity ):
    m_stream( s ),
    point_calculator_( calculator ),
    m_to( to ),
    m_raw_intensity(raw_intensity)
{
    if( from ) { while( m_stream->scan() < *from ) { m_stream->skip_scan(); } }
}

/// read and convert one point from the stream
/// @return false if end of stream is reached
bool velodyne_stream::read()
{
    if( m_to && m_stream->scan() > *m_to ) { return false; }
    const velodyne::laser_return* r = m_stream->read();
    if( r == NULL ) { return false; }
    m_point.timestamp = r->timestamp;
    m_point.id = r->id;
    m_point.intensity = m_raw_intensity ? r->intensity : ( point_calculator_->intensity( m_point.id, r->intensity, r->range ) * 255 ); // multiply by 255 to keep backward compatible with the old format
    m_point.valid = !comma::math::equal( r->range, 0 ); // quick and dirty
    m_point.ray = point_calculator_->ray( m_point.id, r->range, r->azimuth );
    m_point.range = point_calculator_->range( m_point.id, r->range );
    m_point.azimuth = point_calculator_->azimuth( m_point.id, r->azimuth );
    m_point.scan = m_stream->scan();
    m_point.valid_scan=m_stream->is_scan_valid();
    return true;
}

} // namespace snark {

namespace snark { namespace velodyne {

db_calculator::db_calculator( const velodyne::hdl64::db& db ) : db( db ) {}

std::pair< ::Eigen::Vector3d, ::Eigen::Vector3d > db_calculator::ray( unsigned int laser, double range, double angle ) const { return db.lasers[ laser ].ray( range, angle ); }

::Eigen::Vector3d db_calculator::point( unsigned int laser, double range, double angle ) const { return db.lasers[ laser ].point( range, angle ); }

double db_calculator::range( unsigned int laser, double range ) const { return db.lasers[ laser ].range( range ); }

double db_calculator::azimuth( unsigned int laser, double azimuth ) const { return db.lasers[ laser ].azimuth( azimuth ); }

double db_calculator::intensity( unsigned int laser, unsigned char intensity, double distance ) const { return db.lasers[ laser ].intensity( intensity, distance ); }

} } // namespace snark { namespace velodyne {
