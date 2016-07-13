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


#ifndef SNARK_VISITING_TRAITS_H_
#define SNARK_VISITING_TRAITS_H_

#include <comma/visiting/apply.h>
#include <comma/visiting/visit.h>
#include <snark/math/interval.h>
#include "../math/range_bearing_elevation.h"
#include "../math/frame_transforms.h"
#include "../math/roll_pitch_yaw.h"
#include "eigen.h"

namespace comma { namespace visiting {

/// visiting traits
template <>
struct traits< snark::range_bearing_elevation >
{
    /// const visiting
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::range_bearing_elevation& p, Visitor& v )
    {
        v.apply( "range", p.range() );
        v.apply( "bearing", p.bearing() );
        v.apply( "elevation", p.elevation() );
    }

    /// visiting
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::range_bearing_elevation& p, Visitor& v )
    {
        double r = p.r();
        double b = p.b();
        double e = p.e();
        v.apply( "range", r );
        v.apply( "bearing", b );
        v.apply( "elevation", e );
        p = snark::range_bearing_elevation( r, b, e );
    }
};

template <>
struct traits< snark::bearing_elevation >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::bearing_elevation& p, Visitor& v )
    {
        v.apply( "bearing", p.bearing() );
        v.apply( "elevation", p.elevation() );
    }

    template < typename Key, class Visitor > static void visit( const Key&, snark::bearing_elevation& p, Visitor& v )
    {
        double b = p.b();
        double e = p.e();
        v.apply( "bearing", b );
        v.apply( "elevation", e );
        p = snark::bearing_elevation( b, e );
    }
};

template <>
struct traits< snark::roll_pitch_yaw >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::roll_pitch_yaw& p, Visitor& v )
    {
        v.apply( "roll", p.roll() );
        v.apply( "pitch", p.pitch() );
        v.apply( "yaw", p.yaw() );
    }

    template < typename Key, class Visitor > static void visit( const Key&, snark::roll_pitch_yaw& t, Visitor& v )
    {
        double r = t.roll();
        double p = t.pitch();
        double y = t.yaw();
        v.apply( "roll", r );
        v.apply( "pitch", p );
        v.apply( "yaw", y );
        t = snark::roll_pitch_yaw( r, p, y );
    }
};

template < typename T, unsigned int D >
struct traits< snark::math::closed_interval< T, D > >
{
    /// const visiting
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::math::closed_interval< T, D >& p, Visitor& v )
    {
        v.apply( "min", p ? p.min() : Eigen::Matrix< T, D, 1 >() ); // quick and dirty
        v.apply( "max", p ? p.max() : Eigen::Matrix< T, D, 1 >() ); // quick and dirty
    }

    /// visiting
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::math::closed_interval< T, D >& p, Visitor& v )
    {
        Eigen::Matrix< T, D, 1 > min;
        if( p ) { min = p.min(); } // quick and dirty
        v.apply( "min", min );
        Eigen::Matrix< T, D, 1 > max;
        if( p ) { max = p.max(); } // quick and dirty
        v.apply( "max", max );
        p = snark::math::closed_interval< T, D >( min, max );
    }
};

template <> struct traits< snark::frame_transforms::tr_transform >
{
    template< typename K, typename V > static void visit( const K& k, snark::frame_transforms::tr_transform& t, V& v )
    {
        v.apply( "translation", t.translation );
        v.apply( "rotation", t.rotation );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::frame_transforms::tr_transform& t, V& v )
    {
        v.apply( "translation", t.translation );
        v.apply( "rotation", t.rotation );
    }
};

template <> struct traits< snark::frame_transforms::transform >
{
    template< typename K, typename V > static void visit( const K& k, snark::frame_transforms::transform& t, V& v )
    {
        v.apply( "translation", t.translation );
        v.apply( "rotation", t.rotation );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::frame_transforms::transform& t, V& v )
    {
        v.apply( "translation", t.translation );
        v.apply( "rotation", t.rotation );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_VISITING_TRAITS_H_

