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

#ifndef SNARK_SENSORS_HOKUYO_TRAITS_H 
#define SNARK_SENSORS_HOKUYO_TRAITS_H 
#include "message.h"
#include "output.h"

namespace hok = snark::hokuyo;

namespace comma { namespace visiting {

/// Does not work when template is partialially specialised with < int N >
template <  > 
struct traits< boost::array< typename comma::packed::scip_3chars_t, 11 > >
{

    template< typename K, typename V > static void visit( const K& k, const boost::array< comma::packed::scip_3chars_t, 11 >& t, V& v )
    {
        for( std::size_t i=0; i<11; ++i )
        {
            v.apply( boost::lexical_cast< std::string >( i ).c_str(), (t[i])() ); 
        }
    }
};

template < > struct traits< hok::data_point >
{
    template< typename K, typename V > static void visit( const K& k, hok::data_point& t, V& v )
    {
        v.apply( "timestamp", t.timestamp );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
        v.apply( "range", t.range );
        v.apply( "bearing", t.bearing );
        v.apply( "elevation", t.elevation );
        v.apply( "intensity", t.intensity );
    }
    
    template< typename K, typename V > static void visit( const K& k, const hok::data_point& t, V& v )
    {
        v.apply( "timestamp", t.timestamp );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
        v.apply( "range", t.range );
        v.apply( "bearing", t.bearing );
        v.apply( "elevation", t.elevation );
        v.apply( "intensity", t.intensity );
    }
};
    

    
} } // namespace comma { namespace visitting {

#endif // SNARK_SENSORS_HOKUYO_TRAITS_H 