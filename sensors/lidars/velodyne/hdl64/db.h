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

#ifndef SNARK_SENSORS_VELODYNE_DB_H_
#define SNARK_SENSORS_VELODYNE_DB_H_

#include <boost/array.hpp>
#include <comma/base/types.h>
#include <Eigen/Core>
#include "../impl/serializable_db.h"
#include <boost/optional.hpp>

namespace snark {  namespace velodyne { namespace hdl64 {

struct db
{
    struct laser_data
    {
        struct angle
        {
            double value;
            double sin;
            double cos;

            angle();
            angle( const angle& rhs );
            angle( double v );
            angle( double v, double s, double c );
        };

        struct corrention_angles_type
        {
            angle rotational;
            angle vertical;
        };
        
        typedef Eigen::Vector2d distance_correction_t;
        
        //for intensity correction by distance
        struct intensity_correction_t
        {
            double min_intensity;
            double max_intensity;
            double focal_distance;
            double focal_slope;
            intensity_correction_t(double a, double b, double c,double d):min_intensity(a),max_intensity(b),focal_distance(c),focal_slope(d){}
            double calc(double intensity, double distance) const;
        };

        double horizontal_offset;

        double vertical_offset;

        double distance_correction;

        corrention_angles_type correction_angles;

        double elevation;
        
        boost::optional<distance_correction_t> near_distance_correction;
        boost::optional<intensity_correction_t> intensity_correction;

        laser_data();

        laser_data( comma::uint32 id, double horizOffsetCorrection, double vertOffsetCorrection, double distCorrection, angle rotCorrection, angle vertCorrection );

        std::pair< ::Eigen::Vector3d, ::Eigen::Vector3d > ray( double range, double angle ) const;

        ::Eigen::Vector3d point( double range, double angle ) const;

        double range( double range ) const;

        double azimuth( double azimuth ) const;
        //in: intensity between 0..255 and uncorrected distance in meters
        //out: intensity corrected for distance and scaled to 0..1
        double intensity(unsigned char intensity, double distance) const;
    };

    boost::array< laser_data, 64 > lasers;

    db();

    db( const db& db );

    db( const std::string& filename );

    static bool is_upper( unsigned int laser );

    static bool is_lower( unsigned int laser );

    void operator<<( std::istream& s );
    
    unsigned int version;
};

template < class Istream > inline void operator>>( Istream& s, db& db ) { db << s; }

} } } // namespace snark {  namespace velodyne { namespace hdl64 {

#endif // SNARK_SENSORS_VELODYNE_DB_H_
