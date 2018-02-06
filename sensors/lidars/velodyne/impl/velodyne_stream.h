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

#ifndef WIN32
#include <stdlib.h>
#endif
#include <boost/scoped_ptr.hpp>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include "../../../../visiting/eigen.h"
#include "../stream.h"
#include "../hdl64/db.h"
#include "calculator.h"

namespace snark {
 
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
    bool valid_scan;
    
    velodyne_point(): id( 0 ), intensity( 0 ), ray( std::make_pair( ::Eigen::Vector3d::Zero(), ::Eigen::Vector3d::Zero() ) ), azimuth( 0 ), range( 0 ), valid( false ), scan( 0 ), valid_scan(true) {}
};

namespace velodyne {

struct db_calculator : public calculator
{
    const velodyne::hdl64::db& db;
    
    db_calculator( const velodyne::hdl64::db& db );
    std::pair< ::Eigen::Vector3d, ::Eigen::Vector3d > ray( unsigned int laser, double range, double angle ) const;
    ::Eigen::Vector3d point( unsigned int laser, double range, double angle ) const;
    double range( unsigned int laser, double range ) const;
    double azimuth( unsigned int laser, double azimuth ) const;
    double intensity( unsigned int laser, unsigned char intensity, double distance ) const;
};

} // namespace velodyne {

class velodyne_stream
{   
    public:    
        velodyne_stream( velodyne::stream* s
                       , velodyne::calculator* calculator
                       , boost::optional< std::size_t > from = boost::optional< std::size_t >()
                       , boost::optional< std::size_t > to = boost::optional< std::size_t >()
                       , bool raw_intensity = false );

        bool read();
        const velodyne_point& point() const { return m_point; }

    private:
        boost::scoped_ptr< velodyne::stream > m_stream;
        boost::scoped_ptr< velodyne::calculator > point_calculator_;
        velodyne_point m_point;
        boost::optional< std::size_t > m_to;
        bool m_raw_intensity;
};

} // namespace snark

// todo: move to traits.h
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
