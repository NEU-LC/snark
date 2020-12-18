// Copyright (c) 2020 Mission Systems Pty Ltd

#pragma once

#include "types.h"
#include <comma/visiting/traits.h>

namespace comma { namespace visiting {

template <> struct traits< snark::innovusion::output_data_t >
{
    template< typename Key, class Visitor >
    static void visit( const Key&, snark::innovusion::output_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "block", t.block_id );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
        v.apply( "radius", t.radius );
        v.apply( "value", t.value );
    }

    template< typename Key, class Visitor >
    static void visit( const Key&, const snark::innovusion::output_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "block", t.block_id );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
        v.apply( "radius", t.radius );
        v.apply( "value", t.value );
    }
};

} } // namespace comma { namespace visiting {
