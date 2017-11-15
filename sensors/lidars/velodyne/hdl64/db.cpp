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

namespace snark {  namespace velodyne { namespace hdl64 {

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
    double distance_original = distance; // why is DistLSB factor missing?
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
    double dist_coorection_lateral=distance_correction;
    double dist_coorection_forward=distance_correction;
    if(near_distance_correction && distance_original < 25.00)
    {
        double xx = std::abs( distanceXYProjection * correctedangleSin );
        double yy = std::abs( distanceXYProjection * correctedangleCos );
        // hard coded numbers are two point calibration coordinates, from velodyne source/documentation
        dist_coorection_lateral = (distance_correction - near_distance_correction->x())*(xx-2.40)/(25.04-2.40) + near_distance_correction->x();
        dist_coorection_forward = (distance_correction - near_distance_correction->y())*(yy-1.93)/(25.04-1.93) + near_distance_correction->y();
    }
    ray.second.x() = static_cast< double >( (distance_original+dist_coorection_forward) * correction_angles.vertical.cos * correctedangleCos );
    ray.second.y() = static_cast< double >( (distance_original+dist_coorection_lateral) * correction_angles.vertical.cos * correctedangleSin );
    ray.second.z() = static_cast< double >( distance * correction_angles.vertical.sin );
    // laser reading position relative to the velodyne base
    ray.second += ray.first;
    return ray;
}

double db::laser_data::intensity(unsigned char intensity, double distance) const
{
    if (intensity_correction)
    {
        return intensity_correction->calc(double(intensity), distance);
    }
    return double(intensity)/255;
}

template<typename T>
inline T square(T x){return x*x;}

double db::laser_data::intensity_correction_t::calc(double intensity, double distance) const
{
    // this only works when DistLSB is 0.2 (2 mm resolution)
    //max distance in meters
    static const double max_distance= 65535 / 500; 
    //focal_offset on 0..1 scale
    double focal_offset = square(1-focal_distance/131.00);
    intensity+=256*focal_slope*std::abs(focal_offset - square(1-distance/max_distance));
    intensity=std::max(intensity, min_intensity);
    intensity=std::min(intensity, max_intensity);
    //scale to 0 to 1
    return (intensity - min_intensity)/(max_intensity-min_intensity);
}

static db::laser_data laserDataFromSerializable( const impl::serializable_db& serializable, unsigned int i )
{
    // correction angles and offsets are represented in our system of coordinates:
    // x axis: pointing from us, if we look from the back of velodyne
    // y axis: 90 degrees clockwise from x axis
    // z axis: vertical, pointing down
    // positive rotation angle: clockwise (from x to y)
    const impl::px_type& px = serializable.points_()[ i ].px;
    db::laser_data laser(   i
                         // velodyne: from velodyne source code snippet, positive seems to be to the left
                         // ours: positive it to the right
                         , -px.horizOffsetCorrection_ / 100.0
                         // velodyne: positive is upwards, if seen from the back of the laser
                         // ours: positive is downwards
                         , -px.vertOffsetCorrection_ / 100.0
                         // distance correction: velodyne and ours are the same
                         , px.distCorrection_ / 100.0
                         // velodyne: positive is counter-clockwise, if seen from the back of the laser
                         // ours: positive is clockwise
                         , db::laser_data::angle( -px.rotCorrection_ )
                         // velodyne: positive is upwards, if seen from the back of the laser
                         // ours: positive is downwards
                         , db::laser_data::angle( -px.vertCorrection_  ) );
    if (px.version_ > 0)
    {
        laser.near_distance_correction = db::laser_data::distance_correction_t(px.distCorrectionX_/100.0, px.distCorrectionY_/100.0);
        laser.intensity_correction = db::laser_data::intensity_correction_t(serializable.minIntensity_()[ i ], serializable.maxIntensity_()[ i ], px.focalDistance_/100.0, px.focalSlope_);
    }
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
    version = serializable.points_()[0].px.version_;
    for( std::size_t i = 0; i < lasers.size(); ++i ) { lasers[i] = laserDataFromSerializable( serializable, i ); }
}

} } } // namespace snark {  namespace velodyne { namespace hdl64 {
