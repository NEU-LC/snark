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

#include "camera.h"
#include "../../traits.h"

namespace comma { namespace visiting {

template <> struct traits< snark::graphics::qopengl::camera_transform >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::qopengl::camera_transform& p, Visitor& v )
    {
        QVector3D w;
        w=p.get_position(); v.apply( "position", w ); p.set_position(w);
        w=p.get_orientation(); v.apply( "orientation", w ); p.set_orientation(w);
        w=p.center; v.apply( "center", p.center ); p.set_center(w);
        v.apply( "up", p.up );
        v.apply( "orthographic", p.orthographic );
        v.apply( "near_plane", p.near_plane );
        v.apply( "far_plane", p.far_plane );
        v.apply( "field_of_view", p.field_of_view );
        p.update_projection();
    }
    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::qopengl::camera_transform& p, Visitor& v )
    {
        v.apply( "position", p.get_position() );
        v.apply( "orientation", p.get_orientation() );
        v.apply( "center", p.center );
        v.apply( "up", p.up );
        v.apply( "orthographic", p.orthographic );
        v.apply( "near_plane", p.near_plane );
        v.apply( "far_plane", p.far_plane );
        v.apply( "field_of_view", p.field_of_view );
    }
};

} } // namespace comma { namespace visiting {
    
