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

#ifndef SNARK_MATH_SPHERICAL_GEOMETRY_TRAITS_H_
#define SNARK_MATH_SPHERICAL_GEOMETRY_TRAITS_H_

#include <comma/visiting/traits.h>
#include <snark/math/angle.h>
#include "./coordinates.h"

namespace comma { namespace visiting {

/// assume that we ALWAYS use radians inside of our system
/// and input/output ALWAYS is degrees
/// @todo by right we should use boost::units
template <> struct traits< snark::spherical::coordinates > // quick and dirty?
{
    template< typename K, typename V > static void visit( const K&, snark::spherical::coordinates& t, V& v )
    {
        double d = snark::math::degrees( snark::math::radians( t.latitude ) ).value;
        v.apply( "latitude", d );
        t.latitude = snark::math::radians( snark::math::degrees( d ) ).value;
        d = snark::math::degrees( snark::math::radians( t.longitude ) ).value;
        v.apply( "longitude", d );
        t.longitude = snark::math::radians( snark::math::degrees( d ) ).value;
    }

    template< typename K, typename V > static void visit( const K&, const snark::spherical::coordinates& t, V& v )
    {
        double d = snark::math::degrees( snark::math::radians( t.latitude ) ).value;
        v.apply( "latitude", d );
        d = snark::math::degrees( snark::math::radians( t.longitude ) ).value;
        v.apply( "longitude", d );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_MATH_SPHERICAL_GEOMETRY_TRAITS_H_
