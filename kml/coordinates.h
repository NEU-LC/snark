#ifndef SNARK_KML_COORDINATES_H_
#define SNARK_KML_COORDINATES_H_

#include <sstream>
#include <string>
#include <vector>

namespace snark { namespace kml {

struct coordinates
{
    double latitude;
    double longitude;

    coordinates() : latitude( 0 ), longitude( 0 ) {}
    coordinates( double latitude, double longitude ) : latitude( latitude ), longitude( longitude ) {}
};

struct position
{
    kml::coordinates coordinates;
    double altitude;

    position() : altitude( 0 ) {}
    position( double latitude, double longitude, double altitude ) : coordinates( latitude, longitude ), altitude( altitude ) {}
};

std::string as_string( const kml::coordinates& c );

std::string as_string( const kml::position& p );

std::string as_string( const std::vector< kml::position >& v );

} } // namespace snark { namespace kml {

#endif // SNARK_KML_COORDINATES_H_
