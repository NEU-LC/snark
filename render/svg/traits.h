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
