#include <cassert>
#include <cmath>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "./focus.h"

namespace snark {  namespace velodyne { namespace thin {

sector::sector() : bearing( comma::math::interval< double >( -180.0, 180.0 ), 0.0 ), ken( 0 ), range( 0 ) {}

sector::sector( double bearing, double ken, double range )
    : bearing( comma::math::interval< double >( -180.0, 180.0 ), bearing )
    , ken( ken )
    , range( range )
{
}

bool sector::has( double r, double b, double ) const
{
    if( !comma::math::equal( range, 0 ) && comma::math::less( range, r ) ) { return false; }
    comma::math::cyclic< double > a( comma::math::interval< double >( -180.0, 180.0 ), b );
    double diff = std::abs( ( bearing - a )() );
    return comma::math::less( diff, ken / 2 );
}

double sector::coverage() const { return comma::math::equal( range, 0 ) ? ken / 360 : ( range / 30 ) * ( ken / 360 ); } // quick and dirty

} } } // namespace snark {  namespace velodyne { namespace thin {
