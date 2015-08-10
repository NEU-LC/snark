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


#ifndef SNARK_IMAGING_CAMERA_TRAITS_H_
#define SNARK_IMAGING_CAMERA_TRAITS_H_

#include "../../visiting/eigen.h"
#include "config.h"

namespace comma { namespace visiting {

template <> struct traits< snark::camera::config::distortion_t::radial_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::camera::config::distortion_t::radial_t& p, Visitor& v )
    {
        v.apply( "k1", p.k1 );
        v.apply( "k2", p.k2 );
        v.apply( "k3", p.k3 );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const snark::camera::config::distortion_t::radial_t& p, Visitor& v )
    {
        v.apply( "k1", p.k1 );
        v.apply( "k2", p.k2 );
        v.apply( "k3", p.k3 );
    }
};

template <> struct traits< snark::camera::config::distortion_t::tangential_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::camera::config::distortion_t::tangential_t& p, Visitor& v )
    {
        v.apply( "p1", p.p1 );
        v.apply( "p2", p.p2 );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const snark::camera::config::distortion_t::tangential_t& p, Visitor& v )
    {
        v.apply( "p1", p.p1 );
        v.apply( "p2", p.p2 );
    }
};
    
template <> struct traits< snark::camera::config::distortion_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::camera::config::distortion_t& p, Visitor& v )
    {
        v.apply( "radial", p.radial );
        v.apply( "tangential", p.tangential );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const snark::camera::config::distortion_t& p, Visitor& v )
    {
        v.apply( "radial", p.radial );
        v.apply( "tangential", p.tangential );
    }
};
    
template <> struct traits< snark::camera::config >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::camera::config& p, Visitor& v )
    {
        v.apply( "sensor_size", p.image_size );
        v.apply( "image_size", p.image_size );
        v.apply( "focal_length", p.focal_length );
        v.apply( "principal_point", p.principal_point );
        v.apply( "distortion", p.distortion );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const snark::camera::config& p, Visitor& v )
    {
        v.apply( "sensor_size", p.image_size );
        v.apply( "image_size", p.image_size );
        v.apply( "focal_length", p.focal_length );
        v.apply( "principal_point", p.principal_point );
        v.apply( "distortion", p.distortion );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_IMAGING_CAMERA_TRAITS_H_
