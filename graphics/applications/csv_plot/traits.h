// Copyright (c) 2021 Vsevolod Vlaskine

/// @author Vsevolod Vlaskine

#pragma once

#include <comma/csv/traits.h>
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
        v.apply( "series", t.series );
        v.apply( "block", t.block );
        for( unsigned int i = 1; i < t.series.size(); ++i ) // todo: lots of copying, watch performance; hacky for traits in libraries, but ok for applications where traits have limited use
        {
            if( !t.series[i].x ) { t.series[i].x = *t.series[0].x; }
            if( !t.series[i].y ) { t.series[i].y = *t.series[0].y; }
            if( !t.series[i].z ) { t.series[i].z = *t.series[0].z; }
        }
    }
    template< typename K, typename V > static void visit( const K& k, const snark::graphics::plotting::record& t, V& v )
    {
        v.apply( "t", t.t );
        v.apply( "series", t.series );
        v.apply( "block", t.block );
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
