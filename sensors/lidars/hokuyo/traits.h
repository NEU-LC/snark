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

#pragma once

#include "message.h"
#include "output.h"
#include "detail/scip2.h"

namespace comma { namespace visiting {

/// Does not work when template is partialially specialised with < int N >
template <> 
struct traits< boost::array< typename comma::packed::scip_3chars_t, 11 > >
{

    template< typename K, typename V > static void visit( const K& k, const boost::array< comma::packed::scip_3chars_t, 11 >& t, V& v ) // todo: most certainly reimplement properly
    {
        for( std::size_t i=0; i<11; ++i )
        {
            v.apply( boost::lexical_cast< std::string >( i ).c_str(), (t[i])() ); 
        }
    }
};

template <> struct traits< snark::hokuyo::data_point >
{
    template< typename K, typename V > static void visit( const K& k, snark::hokuyo::data_point& t, V& v )
    {
        v.apply( "t", t.t );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
        v.apply( "range", t.range );
        v.apply( "bearing", t.bearing );
        v.apply( "elevation", t.elevation );
        v.apply( "intensity", t.intensity );
    }
    
    template< typename K, typename V > static void visit( const K& k, const snark::hokuyo::data_point& t, V& v )
    {
        v.apply( "t", t.t );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
        v.apply( "range", t.range );
        v.apply( "bearing", t.bearing );
        v.apply( "elevation", t.elevation );
        v.apply( "intensity", t.intensity );
    }
};

template < > struct traits< snark::hokuyo::scip2_device::output_t >
{
    template< typename K, typename V > static void visit( const K& k, snark::hokuyo::scip2_device::output_t& t, V& v )
    {
        v.apply( "t", t.t );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
        v.apply( "block", t.block );
        v.apply( "range", t.range );
        v.apply( "bearing", t.bearing );
        v.apply( "elevation", t.elevation );
    }
    
    template< typename K, typename V > static void visit( const K& k, const snark::hokuyo::scip2_device::output_t& t, V& v )
    {
        v.apply( "t", t.t );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
        v.apply( "block", t.block );
        v.apply( "range", t.range );
        v.apply( "bearing", t.bearing );
        v.apply( "elevation", t.elevation );
    }
};

} } // namespace comma { namespace visiting {
