// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
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

/// @author Navid Pirmarzdashti

#pragma once

#include "ocular.h"
#include "../../visiting/eigen.h"
#include <comma/csv/traits.h>

namespace comma { namespace visiting {
   
template <> struct traits< ocular::ocular_rbe_obs_t >
{
    template< typename K, typename V > static void visit( const K& k, ocular::ocular_rbe_obs_t& p, V& v )
    {
        v.apply( "azimuth", p.azimuth );
        v.apply( "elevation", p.elevation );
        v.apply( "range", p.range );
        //v.apply( "timestamp", p.timestamp );
        v.apply( "amplitude", p.amplitude );
        v.apply( "reflectance", p.reflectance );
        v.apply( "pulse_shape_deviation", p.pulseShapeDeviation );
    }
    template< typename K, typename V > static void visit( const K& k, const ocular::ocular_rbe_obs_t& p, V& v )
    {
        v.apply( "azimuth", p.azimuth );
        v.apply( "elevation", p.elevation );
        v.apply( "range", p.range );
        //v.apply( "timestamp", p.timestamp );
        v.apply( "amplitude", p.amplitude );
        v.apply( "reflectance", p.reflectance );
        v.apply( "pulse_shape_deviation", p.pulseShapeDeviation );
    }
};

template <> struct traits< snark::ocular::roboteye::point_t >
{
    template< typename K, typename V > static void visit( const K& k, snark::ocular::roboteye::point_t& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "point", p.point );
    }
    template< typename K, typename V > static void visit( const K& k, const snark::ocular::roboteye::point_t& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "point", p.point );
    }
};

template <> struct traits< snark::ocular::roboteye::position_t >
{
    template < typename K, typename V > static void visit( const K& key, snark::ocular::roboteye::position_t& t, V& v )
    {
        v.apply( "pan", t.pan );
        v.apply( "tilt", t.tilt );
    }

    template < typename K, typename V > static void visit( const K& key, const snark::ocular::roboteye::position_t& t, V& v )
    {
        v.apply( "pan", t.pan );
        v.apply( "tilt", t.tilt );
    }
};

template <> struct traits< snark::ocular::roboteye::region_scan >
{
    template< typename K, typename V > static void visit( const K& k, snark::ocular::roboteye::region_scan& p, V& v )
    {
        v.apply( "azimuth_rate", p.azimuth_rate );
        v.apply( "azimuth_min", p.azimuth_min );
        v.apply( "azimuth_max", p.azimuth_max );
        v.apply( "elevation_min", p.elevation_min );
        v.apply( "elevation_max", p.elevation_max );
        v.apply( "scan_lines", p.scan_lines );
    }
    template< typename K, typename V > static void visit( const K& k, const snark::ocular::roboteye::region_scan& p, V& v )
    {
        v.apply( "azimuth_rate", p.azimuth_rate );
        v.apply( "azimuth_min", p.azimuth_min );
        v.apply( "azimuth_max", p.azimuth_max );
        v.apply( "elevation_min", p.elevation_min );
        v.apply( "elevation_max", p.elevation_max );
        v.apply( "scan_lines", p.scan_lines );
    }
};

} } // namespace comma { namespace visiting {
