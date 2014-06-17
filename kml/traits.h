#ifndef SNARK_KML_TRAITS_H_
#define SNARK_KML_TRAITS_H_

#include <sstream>
#include <comma/base/exception.h>
#include <comma/visiting/traits.h>
#include "./coordinates.h"
#include "./document.h"
#include "./line_string.h"
#include "./placemark.h"
#include "./point.h"

namespace comma { namespace visiting {

template <> struct traits< snark::kml::placemark >
{
    template< typename K, typename V > static void visit( const K&, const snark::kml::placemark& t, V& v )
    {
        v.apply( "name", t.name );
        v.apply( "description", t.description );
        v.apply( "styleUrl", t.style_url );
        v.apply( "altitude_mode", t.altitude_mode );
        v.apply( "Point", t.point );
        v.apply( "LineString", t.line_string );
    }
};

template <> struct traits< snark::kml::point >
{
    template< typename K, typename V > static void visit( const K&, const snark::kml::point& t, V& v )
    {
        v.apply( "coordinates", snark::kml::as_string( t.coordinates ) );
        v.apply( "extrude", t.extrude );
    }
};

template <> struct traits< snark::kml::line_string >
{
    template< typename K, typename V > static void visit( const K&, const snark::kml::line_string& t, V& v )
    {
        v.apply( "altitude_mode", t.altitude_mode );
        v.apply( "coordinates", snark::kml::as_string( t.coordinates ) ); // quick and dirty
    }
};

template <> struct traits< snark::kml::document > // quick and dirty?
{
    template< typename K, typename V > static void visit( const K&, snark::kml::document& t, V& v )
    {
        v.apply( "Placemark", t.placemarks );
    }

    template< typename K, typename V > static void visit( const K&, const snark::kml::document& t, V& v )
    {
        v.apply( "Placemark", t.placemarks );
    }
};

template <> struct traits< snark::kml::coordinates >
{
    template< typename K, typename V > static void visit( const K&, snark::kml::coordinates& t, V& v )
    {
        v.apply( "latitude", t.latitude );
        v.apply( "longitude", t.longitude );
    }

    template< typename K, typename V > static void visit( const K&, const snark::kml::coordinates& t, V& v )
    {
        v.apply( "latitude", t.latitude );
        v.apply( "longitude", t.longitude );
    }
};

template <> struct traits< snark::kml::position >
{
    template< typename K, typename V > static void visit( const K&, snark::kml::position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "altitude", t.altitude );
    }

    template< typename K, typename V > static void visit( const K&, const snark::kml::position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "altitude", t.altitude );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_KML_TRAITS_H_
