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


#ifndef SNARK_SENSORS_VELODYNE_VELODYNESTREAM_H_
#define SNARK_SENSORS_VELODYNE_VELODYNESTREAM_H_

#ifndef WIN32
#include <stdlib.h>
#endif
#include <boost/scoped_ptr.hpp>
#include <comma/csv/stream.h>
#include <snark/visiting/eigen.h>
#include "../stream.h"
#include "../db.h"
#include "calculator.h"

namespace snark {

/// processed velodyne point    
struct velodyne_point
{
    boost::posix_time::ptime timestamp;
    comma::uint32 id;
    comma::uint32 intensity;
    std::pair< ::Eigen::Vector3d, ::Eigen::Vector3d > ray;
    double azimuth;
    double range;
    bool valid;
    comma::uint32 scan;
};

namespace velodyne {

struct db_calculator : public calculator
{
    const velodyne::db& db;
    
    db_calculator( const velodyne::db& db );
    std::pair< ::Eigen::Vector3d, ::Eigen::Vector3d > ray( unsigned int laser, double range, double angle ) const;
    ::Eigen::Vector3d point( unsigned int laser, double range, double angle ) const;
    double range( unsigned int laser, double range ) const;
    double azimuth( unsigned int laser, double azimuth ) const;
    double intensity( unsigned int laser, unsigned char intensity, double distance ) const;
};

} // namespace velodyne {

/// convert stream of raw velodyne data into velodyne points
template < typename S, typename Traits = velodyne::db_calculator >
class velodyne_stream
{   
public:    
    velodyne_stream( velodyne::calculator* calculator
                  , bool outputInvalidpoints
                  , boost::optional< std::size_t > from = boost::optional< std::size_t >(), boost::optional< std::size_t > to = boost::optional< std::size_t >()
                  , bool raw_intensity = false
                  , bool legacy = false);

    template < typename P >
    velodyne_stream( const P& p
                  , velodyne::calculator* calculator
                  , bool outputInvalidpoints
                  , boost::optional< std::size_t > from = boost::optional< std::size_t >(), boost::optional< std::size_t > to = boost::optional< std::size_t >() 
                  , bool raw_intensity = false
                  , bool legacy = false);

    bool read();
    const velodyne_point& point() const { return m_point; }

private:
    velodyne::stream< S > m_stream;
    boost::scoped_ptr< velodyne::calculator > point_calculator_;
    velodyne_point m_point;
    boost::optional< std::size_t > m_to;
    bool m_raw_intensity;
};

template < typename S, typename Traits >
velodyne_stream< S, Traits >::velodyne_stream ( velodyne::calculator* calculator, bool outputInvalidpoints
                    , boost::optional< std::size_t > from
                    , boost::optional< std::size_t > to 
                    , bool raw_intensity
                    , bool legacy):
    m_stream( new S, outputInvalidpoints, legacy ),
    point_calculator_( calculator ),
    m_to( to ),
    m_raw_intensity(raw_intensity)
{
    if( from ) { while( m_stream.scan() < *from ) { m_stream.skip_scan(); } }
}

template < typename S, typename Traits >
template < typename P >
velodyne_stream< S, Traits >::velodyne_stream ( const P& p, velodyne::calculator* calculator, bool outputInvalidpoints
                    , boost::optional< std::size_t > from
                    , boost::optional< std::size_t > to 
                    , bool raw_intensity
                    , bool legacy ):
    m_stream( new S( p ), outputInvalidpoints, legacy ),
    point_calculator_( calculator ),
    m_to( to ),
    m_raw_intensity(raw_intensity)
{
    if( from ) { while( m_stream.scan() < *from ) { m_stream.skip_scan(); } }
}

/// read and convert one point from the stream
/// @return false if end of stream is reached
template < typename S, typename Traits >
bool velodyne_stream< S, Traits >::read()
{
    if( m_to && m_stream.scan() > *m_to ) { return false; }
    const velodyne::laser_return* r = m_stream.read();
    if( r == NULL ) { return false; }
    m_point.timestamp = r->timestamp;
    m_point.id = r->id;
    // multiply by 255 to keep with the old format
    m_point.intensity = m_raw_intensity ? r->intensity : ( point_calculator_->intensity( m_point.id, r->intensity, r->range ) * 255);
    m_point.valid = !comma::math::equal( r->range, 0 ); // quick and dirty
    m_point.ray = point_calculator_->ray( m_point.id, r->range, r->azimuth );
    m_point.range = point_calculator_->range( m_point.id, r->range );
    m_point.scan = m_stream.scan();
    m_point.azimuth = point_calculator_->azimuth( m_point.id, r->azimuth );
    return true;
}

/// specialisation for csv input stream: in this case nothing to convert
template < typename Traits >
class velodyne_stream< comma::csv::input_stream< velodyne_point >, Traits >
{
public:

    velodyne_stream( const comma::csv::options& options ): m_stream( std::cin, options ) {}

    bool read();
    const velodyne_point& point() const { return m_point; }

private:
    comma::csv::input_stream< velodyne_point > m_stream;
    velodyne_point m_point;
};

/// read and forward one point from the stream: nothing to convert as the input is csv
/// @return false if end of stream is reached
template < typename Traits >
inline bool velodyne_stream< comma::csv::input_stream< velodyne_point >, Traits >::read()
{
    const velodyne_point* point = m_stream.read();
    if( !point ) { return false; }
    m_point = *point;
    return true;
}

} // namespace snark

namespace comma { namespace visiting {

template <> struct traits< snark::velodyne_point >
{    
    template < typename K, typename V > static void visit( const K&, snark::velodyne_point& p, V& v )
    {
        v.apply( "t", p.timestamp );
        v.apply( "id", p.id );
        v.apply( "intensity", p.intensity );
        v.apply( "ray", p.ray );
        v.apply( "azimuth", p.azimuth );
        v.apply( "range", p.range );
        v.apply( "valid", p.valid );
        v.apply( "scan", p.scan );
    }
    
    template < typename K, typename V > static void visit( const K&, const snark::velodyne_point& p, V& v )
    {
        v.apply( "t", p.timestamp );
        v.apply( "id", p.id );
        v.apply( "intensity", p.intensity );
        v.apply( "ray", p.ray );
        v.apply( "azimuth", p.azimuth );
        v.apply( "range", p.range );
        v.apply( "valid", p.valid );
        v.apply( "scan", p.scan );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_SENSORS_VELODYNE_VELODYNESTREAM_H_
