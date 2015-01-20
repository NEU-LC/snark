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
        v.apply( "viewbox", t.viewbox );
    }

    template< typename K, typename V > static void visit( const K&, const snark::render::svg::header& t, V& v )
    {
        v.apply( "width", t.width );
        v.apply( "height", t.height );
        v.apply( "viewbox", t.viewbox );
    }
};

template <> struct traits< snark::render::svg::text >
{
    template< typename K, typename V > static void visit( const K&, snark::render::svg::text& t, V& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "value", t.value );
    }

    template< typename K, typename V > static void visit( const K&, const snark::render::svg::text& t, V& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "value", t.value );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_RENDER_SVG_TRAITS_H
