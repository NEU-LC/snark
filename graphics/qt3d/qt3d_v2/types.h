// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
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

#ifndef SNARK_GRAPHICS_QT3D_TYPES_H_
#define SNARK_GRAPHICS_QT3D_TYPES_H_

#include <boost/array.hpp>
#include <Eigen/Core>
#include <QColor>
#include <GL/gl.h>
#include "../../../visiting/eigen.h"

namespace snark { namespace graphics { namespace qt3d {

struct gl_color_t
{
    boost::array< GLfloat, 4 > rgba;

    gl_color_t()
    {
        rgba[0] = 0.0; rgba[1] = 0.0; rgba[2] = 0.0; rgba[3] = 1.0;
    }

    gl_color_t( GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha=1.0f )
    {
        rgba[0] = red; rgba[1] = green; rgba[2] = blue; rgba[3] = alpha;
    }

    gl_color_t( double red, double green, double blue, double alpha )
    {
        rgba[0] = red; rgba[1] = green; rgba[2] = blue; rgba[3] = alpha;
    }

    gl_color_t( int red, int green, int blue, int alpha=255 )
    {
        rgba[0] = red   / 255.0f;
        rgba[1] = green / 255.0f;
        rgba[2] = blue  / 255.0f;
        rgba[3] = alpha / 255.0f;
    }

    gl_color_t( const QColor& color )
    {
        rgba[0] = static_cast< GLfloat >( color.redF() );
        rgba[1] = static_cast< GLfloat >( color.greenF() );
        rgba[2] = static_cast< GLfloat >( color.blueF() );
        rgba[3] = static_cast< GLfloat >( color.alphaF() );
    }

    gl_color_t( const gl_color_t& color ) : rgba( color.rgba ) {}

    GLfloat red() const   { return rgba[0]; }
    GLfloat green() const { return rgba[1]; }
    GLfloat blue() const  { return rgba[2]; }
    GLfloat alpha() const { return rgba[3]; }
};

inline gl_color_t operator+( const gl_color_t& lhs, const gl_color_t& rhs )
{
    return gl_color_t( std::min( lhs.red()   + rhs.red(), 1.0f )
                     , std::min( lhs.green() + rhs.green(), 1.0f )
                     , std::min( lhs.blue()  + rhs.blue(), 1.0f )
                     , std::min( lhs.alpha() + rhs.alpha(), 1.0f ));
}

inline gl_color_t operator*( const gl_color_t& color, double scalar )
{
    return gl_color_t( color.red()   * scalar
                     , color.green() * scalar
                     , color.blue()  * scalar
                     , color.alpha() * scalar );
}

struct vertex_t
{
    Eigen::Vector3f position;
    gl_color_t color;

    vertex_t() {}
    vertex_t( const Eigen::Vector3f& position, const gl_color_t& color )
        : position( position ), color( color ) {}
    vertex_t( const Eigen::Vector3f& position, const QColor& color )
        : position( position ), color( color ) {}
};

} } } // namespace snark { namespace graphics { namespace qt3d {

namespace comma { namespace visiting {

template <> struct traits< snark::graphics::qt3d::gl_color_t >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::qt3d::gl_color_t& p, Visitor& v )
    {
        int red   = 0;
        int green = 0;
        int blue  = 0;
        int alpha = 255;
        v.apply( "r", red );
        v.apply( "g", green );
        v.apply( "b", blue );
        v.apply( "a", alpha );
        p = snark::graphics::qt3d::gl_color_t( red, green, blue, alpha );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::qt3d::gl_color_t& p, Visitor& v )
    {
        v.apply( "r", p.red() );
        v.apply( "g", p.green() );
        v.apply( "b", p.blue() );
        v.apply( "a", p.alpha() );
    }
};

template <> struct traits< snark::graphics::qt3d::vertex_t >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::qt3d::vertex_t& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "color", p.color );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::qt3d::vertex_t& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "color", p.color );
    }
};

} } // namespace comma { namespace visiting {

#endif /*SNARK_GRAPHICS_QT3D_TYPES_H_*/
