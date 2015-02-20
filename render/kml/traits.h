#ifndef SNARK_RENDER_KML_TRAITS_H_
#define SNARK_RENDER_KML_TRAITS_H_

#include <sstream>
#include <comma/base/exception.h>
#include <comma/visiting/traits.h>
#include "coordinates.h"
#include "document.h"
#include "line_string.h"
#include "placemark.h"
#include "point.h"

namespace comma { namespace visiting {

template <> struct traits< snark::render::kml::placemark >
{
    template< typename K, typename V > static void visit( const K&, const snark::render::kml::placemark& t, V& v )
    {
        v.apply( "name", t.name );
        v.apply( "description", t.description );
        v.apply( "styleUrl", t.style_url );
        v.apply( "altitude_mode", t.altitude_mode );
        v.apply( "Point", t.point );
        v.apply( "LineString", t.line_string );
    }
};

template <> struct traits< snark::render::kml::point >
{
    template< typename K, typename V > static void visit( const K&, const snark::render::kml::point& t, V& v )
    {
        v.apply( "coordinates", snark::render::kml::as_string( t.coordinates ) );
        v.apply( "extrude", t.extrude );
    }
};

template <> struct traits< snark::render::kml::line_string >
{
    template< typename K, typename V > static void visit( const K&, const snark::render::kml::line_string& t, V& v )
    {
        v.apply( "altitude_mode", t.altitude_mode );
        v.apply( "coordinates", snark::render::kml::as_string( t.coordinates ) ); // quick and dirty
    }
};

template <> struct traits< snark::render::kml::document > // quick and dirty?
{
    template< typename K, typename V > static void visit( const K&, snark::render::kml::document& t, V& v )
    {
        v.apply( "Placemark", t.placemarks );
    }

    template< typename K, typename V > static void visit( const K&, const snark::render::kml::document& t, V& v )
    {
        v.apply( "Placemark", t.placemarks );
    }
};

template <> struct traits< snark::render::kml::coordinates >
{
    template< typename K, typename V > static void visit( const K&, snark::render::kml::coordinates& t, V& v )
    {
        v.apply( "latitude", t.latitude );
        v.apply( "longitude", t.longitude );
    }

    template< typename K, typename V > static void visit( const K&, const snark::render::kml::coordinates& t, V& v )
    {
        v.apply( "latitude", t.latitude );
        v.apply( "longitude", t.longitude );
    }
};

template <> struct traits< snark::render::kml::position >
{
    template< typename K, typename V > static void visit( const K&, snark::render::kml::position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "altitude", t.altitude );
    }

    template< typename K, typename V > static void visit( const K&, const snark::render::kml::position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "altitude", t.altitude );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_RENDER_KML_TRAITS_H_
