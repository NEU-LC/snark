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