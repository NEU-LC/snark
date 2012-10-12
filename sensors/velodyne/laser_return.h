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

#ifndef SNARK_SENSORS_VELODYNE_LASERRETURN_H_
#define SNARK_SENSORS_VELODYNE_LASERRETURN_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/base/types.h>
#include <comma/visiting/traits.h>

namespace snark { namespace velodyne {

/// velodyne point corresponding to a single laser return
struct laser_return
{
    /// timestamp
    boost::posix_time::ptime timestamp;
    
    /// laser id
    comma::uint32 id;
    
    /// intensity
    unsigned char intensity;
    
    /// range (in metres, without range correction)
    double range;
    
    /// azimuth (in degrees, without angle correction)
    double azimuth;
};

} } // namespace snark  { namespace velodyne {

namespace comma { namespace visiting {

template <> struct traits< snark::velodyne::laser_return >
{
    template < typename Key, class Visitor >
    static void visit( Key k, snark::velodyne::laser_return& t, Visitor& v )
    {
        v.apply( "t", t.timestamp );
        v.apply( "id", t.id );
        v.apply( "intensity", t.intensity );
        v.apply( "range", t.range );
        v.apply( "azimuth", t.azimuth );
    }
    
    template < typename Key, class Visitor >
    static void visit( Key k, const snark::velodyne::laser_return& t, Visitor& v )
    {
        v.apply( "t", t.timestamp );
        v.apply( "id", t.id );
        v.apply( "intensity", t.intensity );
        v.apply( "range", t.range );
        v.apply( "azimuth", t.azimuth );
    }
};

} } // namespace comma { namespace visiting {

#endif /*SNARK_SENSORS_VELODYNE_LASERRETURN_H_*/
