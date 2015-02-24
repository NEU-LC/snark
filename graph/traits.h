// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// @author vsevolod vlaskine

#ifndef SNARK_GRAPH_TRAITS_H_
#define SNARK_GRAPH_TRAITS_H_

#include <comma/base/types.h>
#include <comma/visiting/traits.h>
#include "search_graph.h"

namespace comma { namespace visiting {

template < typename T > struct traits< snark::detail::optional< T > >
{
    template < typename K, typename V > static void visit( const K&, snark::detail::optional< T >& n, V& v )
    { // quick and dirty
        T t;
        if( n ) { t = *n; }
        bool b = n;
        v.apply( "is_set", b );
        v.apply( "value", t ); // quick and dirty: apply anyway for now, otherwise it does not work
        if( b ) { n = t; } else { n.reset(); }
    }

    template < typename K, typename V > static void visit( const K&, const snark::detail::optional< T >& n, V& v )
    {
        v.apply( "is_set", bool( n ) );
        v.apply( "value", *n );
    }
};

template < typename P, typename C > struct traits< typename snark::search_node< P, C > >
{
    template < typename K, typename V > static void visit( const K&, typename snark::search_node< P, C >& n, V& v )
    {
        v.apply( "id", n.id );
        v.apply( "best_parent", n.best_parent );
        v.apply( "value", n.value );
    }

    template < typename K, typename V > static void visit( const K&, const typename snark::search_node< P, C >& n, V& v )
    {
        v.apply( "id", n.id );
        v.apply( "best_parent", n.best_parent );
        v.apply( "value", n.value );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_GRAPH_TRAITS_H_
