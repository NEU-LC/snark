#include "./coordinates.h"

namespace snark { namespace kml {

std::string as_string( const kml::coordinates& c ) { std::ostringstream oss; oss.precision( 12 ); oss << c.longitude << ',' << c.latitude; return oss.str(); }

std::string as_string( const kml::position& p ) { std::ostringstream oss; oss.precision( 12 ); oss << as_string( p.coordinates ) << ',' << p.altitude; return oss.str(); }

std::string as_string( const std::vector< kml::position >& v )
{
    if( v.empty() ) { return ""; }
    std::ostringstream oss;
    oss.precision( 12 );
    for( std::size_t i = 0; i < v.size(); ++i ) { oss << as_string( v[i] ) << std::endl; }
    return oss.str();
}

} } // namespace snark { namespace kml {
