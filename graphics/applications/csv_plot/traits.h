// Copyright (c) 2021 Vsevolod Vlaskine

/// @author Vsevolod Vlaskine

#pragma once

#include <comma/visiting/traits.h>
#include "record.h"
#include "series.h"
#include "stream.h"

namespace comma { namespace visiting {

template <> struct traits< snark::graphics::plotting::point >
{
    template< typename K, typename V > static void visit( const K&, snark::graphics::plotting::point& t, V& v )
    {
        if( t.x ) { v.apply( "x", *t.x ); } // todo? what should be the behaviour in comma::csv::from_ascii on optional?
        if( t.y ) { v.apply( "y", *t.y ); } // todo? what should be the behaviour in comma::csv::from_ascii on optional?
        if( t.z ) { v.apply( "z", *t.z ); } // todo? what should be the behaviour in comma::csv::from_ascii on optional?
    }

    template< typename K, typename V > static void visit( const K&, const snark::graphics::plotting::point& t, V& v )
    {
        if( t.x ) { v.apply( "x", *t.x ); }
        if( t.y ) { v.apply( "y", *t.y ); }
        if( t.z ) { v.apply( "z", *t.z ); }
    }
};

template <> struct traits< snark::graphics::plotting::record >
{
    template< typename K, typename V > static void visit( const K& k, snark::graphics::plotting::record& t, V& v )
    {
        v.apply( "t", t.t );
        comma::visiting::traits< snark::graphics::plotting::point >::visit( k, t, v ); // quick and dirty: t,x,y,z looks better than x,y,z,t
        v.apply( "block", t.block );
        v.apply( "series", t.series );
        for( auto& s: t.series ) // hacky for traits in libraries, but ok for applications where traits have limited use
        {
            if( !s.x ) { s.x = *t.x; }
            if( !s.y ) { s.y = *t.y; }
            if( !s.z ) { s.z = *t.z; }
        }
    }

    template< typename K, typename V > static void visit( const K& k, const snark::graphics::plotting::record& t, V& v )
    {
        v.apply( "t", t.t );
        comma::visiting::traits< snark::graphics::plotting::point >::visit( k, t, v ); // quick and dirty: t,x,y,z looks better than x,y,z,t
        v.apply( "block", t.block );
        v.apply( "series", t.series );
    }
};

template <> struct traits< snark::graphics::plotting::series::config >
{
    template< typename K, typename V > static void visit( const K&, snark::graphics::plotting::series::config& t, V& v )
    {
        v.apply( "chart", t.chart );
        v.apply( "color", t.color_name );
        t.color = QColor( &t.color_name[0] );
        v.apply( "name", t.name );
        v.apply( "scroll", t.scroll );
        v.apply( "shape", t.shape );
        v.apply( "style", t.style );
        v.apply( "title", t.title );
        v.apply( "weight", t.weight );
    }

    template< typename K, typename V > static void visit( const K&, const snark::graphics::plotting::series::config& t, V& v )
    {
        v.apply( "chart", t.chart );
        v.apply( "color", std::string( t.color.name() ) );
        v.apply( "name", t.name );
        v.apply( "scroll", t.scroll );
        v.apply( "shape", t.shape );
        v.apply( "style", t.style );
        v.apply( "title", t.title );
        v.apply( "weight", t.weight );
    }
};

template <> struct traits< snark::graphics::plotting::stream::config_t >
{
    template< typename K, typename V > static void visit( const K&, snark::graphics::plotting::stream::config_t& t, V& v )
    {
        v.apply( "csv", t.csv );
        v.apply( "pass-through", t.pass_through );
        v.apply( "series", t.series );
        v.apply( "size", t.size );
        v.apply( "number-of-series", t.number_of_series );
    }

    template< typename K, typename V > static void visit( const K&, const snark::graphics::plotting::stream::config_t& t, V& v )
    {
        v.apply( "csv", t.csv );
        v.apply( "pass-through", t.pass_through );
        v.apply( "series", t.series );
        v.apply( "size", t.size );
        v.apply( "number-of-series", t.number_of_series );
    }
};

} } // namespace comma { namespace visiting {
