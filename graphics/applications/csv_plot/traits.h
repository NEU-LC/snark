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

/// @author Vsevolod Vlaskine

#ifndef SNARK_GRAPHICS_APPLICATIONS_CSV_PLOT_TRAITS_H_
#define SNARK_GRAPHICS_APPLICATIONS_CSV_PLOT_TRAITS_H_

#include <snark/visiting/eigen.h>
#include "./point.h"
#include "./stream.h"

namespace comma { namespace visiting {

template <> struct traits< QColor >
{
    template< typename K, typename V > static void visit( const K&, QColor& t, V& v )
    {
        int a;
        a = t.red();
        v.apply( "r", a );
        t.setRed( a );
        a = t.green();
        v.apply( "g", a );
        t.setGreen( a );
        a = t.blue();
        v.apply( "b", a );
        t.setBlue( a );
        a = t.alpha();
        v.apply( "a", a );
        t.setAlpha( a );
    }

    template< typename K, typename V > static void visit( const K&, const QColor& t, V& v )
    {
        v.apply( "r", t.red() );
        v.apply( "g", t.green() );
        v.apply( "b", t.blue() );
        v.apply( "a", t.alpha() );
    }
};
    
template <> struct traits< snark::graphics::plotting::point >
{
    template< typename K, typename V > static void visit( const K&, snark::graphics::plotting::point& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "color", t.color );
        v.apply( "block", t.block );
    }

    template< typename K, typename V > static void visit( const K&, const snark::graphics::plotting::point& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "color", t.color );
        v.apply( "block", t.block );
    }
};

template <> struct traits< snark::graphics::plotting::stream::config_t >
{
    template< typename K, typename V > static void visit( const K&, snark::graphics::plotting::stream::config_t& t, V& v )
    {
        v.apply( "csv", t.csv );
        v.apply( "size", t.size );
        v.apply( "color", t.color_name );
        t.color = QColor( &t.color_name[0] );
        v.apply( "shape", t.shape );
        v.apply( "style", t.style );
        v.apply( "weight", t.weight );
    }

    template< typename K, typename V > static void visit( const K&, const snark::graphics::plotting::stream::config_t& t, V& v )
    {
        v.apply( "csv", t.csv );
        v.apply( "size", t.size );
        v.apply( "color", std::string( t.color.name() ) );
        v.apply( "shape", t.shape );
        v.apply( "style", t.style );
        v.apply( "weight", t.weight );
    }
};

} } // namespace comma { namespace visiting {

#endif // #ifndef SNARK_GRAPHICS_APPLICATIONS_CSV_PLOT_TRAITS_H_
