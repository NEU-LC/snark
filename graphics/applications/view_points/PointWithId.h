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

#ifndef SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_POINTWITHID_H_
#define SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_POINTWITHID_H_

#include <comma/base/types.h>
#include <snark/visiting/eigen.h>
#include <Qt3D/qcolor4ub.h>
#include "./ShapeWithId.h"

namespace snark { namespace graphics { namespace View {

struct PointWithId // quick and dirty
{
    PointWithId() : id( 0 ), block( 0 ) {}
    Eigen::Vector3d point;
    Eigen::Vector3d orientation;
    comma::uint32 id;
    comma::uint32 block;
    QColor4ub color;
    std::string label;
    double scalar;
};

} } } // namespace snark { namespace graphics { namespace View {

namespace comma { namespace visiting {

template <> struct traits< snark::graphics::View::PointWithId >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::View::PointWithId& p, Visitor& v )
    {
        v.apply( "point", p.point );
        v.apply( "roll", p.orientation.x() );
        v.apply( "pitch", p.orientation.y() );
        v.apply( "yaw", p.orientation.z() );
        v.apply( "id", p.id );
        v.apply( "block", p.block );
        v.apply( "colour", p.color );
        v.apply( "label", p.label );
        v.apply( "scalar", p.scalar );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::View::PointWithId& p, Visitor& v )
    {
        v.apply( "point", p.point );
        v.apply( "roll", p.orientation.x() );
        v.apply( "pitch", p.orientation.y() );
        v.apply( "yaw", p.orientation.z() );
        v.apply( "id", p.id );
        v.apply( "block", p.block );
        v.apply( "color", p.color );
        v.apply( "label", p.label );
        v.apply( "scalar", p.scalar );
    }
};


} } // namespace comma { namespace visiting {

#endif /*SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_POINTWITHID_H_*/
