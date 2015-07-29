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

/// @author vsevolod vlaskine

#include <cmath>
#include <fstream>
#include <sstream>
#include <boost/archive/tmpdir.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <comma/base/exception.h>
#include "db.h"

namespace snark {  namespace velodyne {

db::db() {}

db::db( const db& rhs ) { operator=( rhs ); }

db::db( const std::string& filename )
{
    std::ifstream ifs( filename.c_str() );
    if( !ifs.good() ) { COMMA_THROW( comma::exception, "failed to open \"" << filename << "\" for reading" ); }
    this->operator<<( ifs );
}

db::laser_data::angle::angle() : value( 0 ), sin( 0 ), cos( 1 ) {}

db::laser_data::angle::angle( const angle& rhs ) { operator=( rhs ); }

db::laser_data::angle::angle( double v )
    : value( v )
    , sin( ::sin( v * M_PI / 180.0 ) )
    , cos( ::cos( v * M_PI / 180.0 ) )
{
}

db::laser_data::angle::angle( double v, double s, double c ) : value( v ), sin( s ), cos( c ) {}

db::laser_data::laser_data() : horizontal_offset( 0 ), vertical_offset( 0 ), distance_correction( 0 ) {}

db::laser_data::laser_data(   comma::uint32 id
                          , double horizOffsetCorrection
                          , double vertOffsetCorrection
                          , double distCorrection
                          , laser_data::angle rotCorrection
                          , laser_data::angle vertCorrection )
    //: id( id )
    : horizontal_offset( horizOffsetCorrection )
    , vertical_offset( vertOffsetCorrection )
    , distance_correction( distCorrection )
{
    correction_angles.rotational = rotCorrection;
    correction_angles.vertical = vertCorrection;
    elevation = std::asin( vertical_offset * correction_angles.vertical.cos
                         + distance_correction * correction_angles.vertical.sin );
}

::Eigen::Vector3d db::laser_data::point( double distance, double angle ) const
{
    return ray( distance, angle ).second;
}

double db::laser_data::range( double range ) const
{
    return range + distance_correction;
}

double db::laser_data::azimuth( double azimuth ) const
{
    double a = azimuth + correction_angles.rotational.value;
    if( a > 360 ) { a -= 360; } else if( a < 0 ) { a += 360; }
    return a;
}

/// @note this give the same result as in "Calibration of a rotating multi-beam Lidar",
///       Naveed Muhammad and Simon Lacroix, with the coordinate system changes
std::pair< ::Eigen::Vector3d, ::Eigen::Vector3d > db::laser_data::ray( double distance, double a ) const
{
    angle angle( a );
    distance += distance_correction;
    // add 90 degrees for our system of coordinates
    //double angleSin( angle.cos ); // could also be added to the nav to velodyne offset
    //double angleCos( -angle.sin );
    double angleSin( angle.sin );
    double angleCos( angle.cos );

    double correctedangleCos( angleCos * correction_angles.rotational.cos - angleSin * correction_angles.rotational.sin );
    double correctedangleSin( angleSin * correction_angles.rotational.cos + angleCos * correction_angles.rotational.sin );
    std::pair< ::Eigen::Vector3d, ::Eigen::Vector3d > ray;
    // laser position
    double vertical_offsetXYProjection( vertical_offset * correction_angles.vertical.sin );
    ray.first.x() = static_cast< double >( -horizontal_offset * correctedangleSin - vertical_offsetXYProjection * correctedangleCos );
    ray.first.y() = static_cast< double >( horizontal_offset * correctedangleCos - vertical_offsetXYProjection * correctedangleSin );
    ray.first.z() = static_cast< double >( vertical_offset * correction_angles.vertical.cos );
    // laser reading position relative to the laser
    double distanceXYProjection( distance * correction_angles.vertical.cos );
    ray.second.x() = static_cast< double >( distanceXYProjection * correctedangleCos );
    ray.second.y() = static_cast< double >( distanceXYProjection * correctedangleSin );
    ray.second.z() = static_cast< double >( distance * correction_angles.vertical.sin );
    // laser reading position relative to the velodyne base
    ray.second += ray.first;
    return ray;
}

static db::laser_data laserDataFromSerializable( const impl::serializable_db& serializable, unsigned int i )
{
    // correction angles and offsets are represented in our system of coordinates:
    // x axis: pointing from us, if we look from the back of velodyne
    // y axis: 90 degrees clockwise from x axis
    // z axis: vertical, pointing down
    // positive rotation angle: clockwise (from x to y)
    db::laser_data laser(   i
                         // velodyne: from velodyne source code snippet, positive seems to be to the left
                         // ours: positive it to the right
                         , -serializable.points_()[ i ].px.horizOffsetCorrection_ / 100.0
                         // velodyne: positive is upwards, if seen from the back of the laser
                         // ours: positive is downwards
                         , -serializable.points_()[ i ].px.vertOffsetCorrection_ / 100.0
                         // distance correction: velodyne and ours are the same
                         , serializable.points_()[ i ].px.distCorrection_ / 100.0
                         // velodyne: positive is counter-clockwise, if seen from the back of the laser
                         // ours: positive is clockwise
                         , db::laser_data::angle( -serializable.points_()[ i ].px.rotCorrection_ )
                         // velodyne: positive is upwards, if seen from the back of the laser
                         // ours: positive is downwards
                         , db::laser_data::angle( -serializable.points_()[ i ].px.vertCorrection_ ) );
    return laser;
}

bool db::is_upper( unsigned int laser ) { return laser < 32; }
bool db::is_lower( unsigned int laser ) { return laser >= 32; }

void db::operator<<( std::istream& s )
{
    impl::serializable_db serializable;
    assert( s.good() );
    boost::archive::xml_iarchive ia( s );
    ia >> boost::serialization::make_nvp( "DB", serializable );
    for( std::size_t i = 0; i < lasers.size(); ++i ) { lasers[i] = laserDataFromSerializable( serializable, i ); }
}

} } // namespace snark {  namespace velodyne {
