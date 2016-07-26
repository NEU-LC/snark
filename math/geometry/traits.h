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

#ifndef SNARK_MATH_GEOMETRY_TRAITS_H_
#define SNARK_MATH_GEOMETRY_TRAITS_H_

#include "../../visiting/eigen.h"
#include "polygon.h"

namespace comma { namespace visiting {

template <> struct traits< snark::convex_polygon >
{
    template< typename K, typename V > static void visit( const K&, snark::convex_polygon& t, V& v ) { v.apply( "corners", t.corners ); }
    template< typename K, typename V > static void visit( const K&, const snark::convex_polygon& t, V& v ) { v.apply( "corners", t.corners ); }
};

template <> struct traits< snark::triangle >
{
    template< typename K, typename V > static void visit( const K&, snark::triangle& t, V& v ) { v.apply( "corners", t.corners ); }
    template< typename K, typename V > static void visit( const K&, const snark::triangle& t, V& v ) { v.apply( "corners", t.corners ); }
};

} } // namespace comma { namespace visiting {

#endif // #ifndef SNARK_MATH_GEOMETRY_TRAITS_H_
