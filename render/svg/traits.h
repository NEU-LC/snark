#ifndef SNARK_RENDER_SVG_TRAITS_H
#define SNARK_RENDER_SVG_TRAITS_H

#include <comma/visiting/traits.h>
#include "svg.h"

namespace comma { namespace visiting {

template <> struct traits< snark::render::svg::circle >
{
    template< typename K, typename V > static void visit( const K&, snark::render::svg::circle& t, V& v )
    {
        v.apply( "x", t.cx );
        v.apply( "y", t.cy );
        v.apply( "r", t.r );
    }

    template< typename K, typename V > static void visit( const K&, const snark::render::svg::circle& t, V& v )
    {
        v.apply( "x", t.cx );
        v.apply( "y", t.cy );
        v.apply( "r", t.r );
    }
};

template <> struct traits< snark::render::svg::line >
{
    template< typename K, typename V > static void visit( const K&, snark::render::svg::line& t, V& v )
    {
        v.apply( "x1", t.x1 );
        v.apply( "y1", t.y1 );
        v.apply( "x2", t.x2 );
        v.apply( "y2", t.y2 );
    }

    template< typename K, typename V > static void visit( const K&, const snark::render::svg::line& t, V& v )
    {
        v.apply( "x1", t.x1 );
        v.apply( "y1", t.y1 );
        v.apply( "x2", t.x2 );
        v.apply( "y2", t.y2 );
    }
};

template <> struct traits< snark::render::svg::point >
{
    template< typename K, typename V > static void visit( const K&, snark::render::svg::point& t, V& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
    }

    template< typename K, typename V > static void visit( const K&, const snark::render::svg::point& t, V& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
    }
};

template <> struct traits< snark::render::svg::header >
{
    template< typename K, typename V > static void visit( const K&, snark::render::svg::header& t, V& v )
    {
        v.apply( "width", t.width );
        v.apply( "height", t.height );
        v.apply( "viewbox_min_x", t.viewbox_min_x );
        v.apply( "viewbox_min_y", t.viewbox_min_y );
        v.apply( "viewbox_width", t.viewbox_width );
        v.apply( "viewbox_height", t.viewbox_height );
    }

    template< typename K, typename V > static void visit( const K&, const snark::render::svg::header& t, V& v )
    {
        v.apply( "width", t.width );
        v.apply( "height", t.height );
        v.apply( "viewbox_min_x", t.viewbox_min_x );
        v.apply( "viewbox_min_y", t.viewbox_min_y );
        v.apply( "viewbox_width", t.viewbox_width );
        v.apply( "viewbox_height", t.viewbox_height );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_RENDER_SVG_TRAITS_H
