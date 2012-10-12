// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#ifndef SNARK_SENSORS_VELODYNE_DB_H_
#define SNARK_SENSORS_VELODYNE_DB_H_

#include <boost/array.hpp>
#include <comma/base/types.h>
#include <Eigen/Core>
#include <snark/sensors/velodyne/impl/serializable_db.h>

namespace snark {  namespace velodyne {

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

        double horizontal_offset;

        double vertical_offset;

        double distance_correction;

        corrention_angles_type correction_angles;

        laser_data();

        laser_data( comma::uint32 id, double horizOffsetCorrection, double vertOffsetCorrection, double distCorrection, angle rotCorrection, angle vertCorrection );

        std::pair< ::Eigen::Vector3d, ::Eigen::Vector3d > ray( double range, double angle ) const;

        ::Eigen::Vector3d point( double range, double angle ) const;

        double range( double range ) const;

        double azimuth( double azimuth ) const;
    };

    boost::array< laser_data, 64 > lasers;

    db();

    db( const db& db );

    db( const std::string& filename );

    static bool is_upper( unsigned int laser );

    static bool is_lower( unsigned int laser );

    void operator<<( std::istream& s );
};

template < class Istream > inline void operator>>( Istream& s, db& db ) { db << s; }

} } // namespace snark {  namespace velodyne {

#endif // SNARK_SENSORS_VELODYNE_DB_H_
