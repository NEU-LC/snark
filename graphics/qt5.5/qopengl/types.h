// Copyright (c) 2016 The University of Sydney

#pragma once

#ifndef Q_MOC_RUN
#include <boost/array.hpp>
#include "../../../visiting/eigen.h"
#endif
#include <Eigen/Core>
#include <QColor>
#include <GL/gl.h>

namespace snark { namespace graphics { namespace qopengl {

struct color_t
{
    boost::array< GLfloat, 4 > rgba;

    color_t() { rgba[0] = 0.0; rgba[1] = 0.0; rgba[2] = 0.0; rgba[3] = 1.0; }

    color_t( GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha=1.0f ) { rgba[0] = red; rgba[1] = green; rgba[2] = blue; rgba[3] = alpha; }

    color_t( double red, double green, double blue, double alpha ) { rgba[0] = red; rgba[1] = green; rgba[2] = blue; rgba[3] = alpha; }

    color_t( int red, int green, int blue, int alpha=255 )
    {
        rgba[0] = red   / 255.0f;
        rgba[1] = green / 255.0f;
        rgba[2] = blue  / 255.0f;
        rgba[3] = alpha / 255.0f;
    }

    color_t( const QColor& color )
    {
        rgba[0] = static_cast< GLfloat >( color.redF() );
        rgba[1] = static_cast< GLfloat >( color.greenF() );
        rgba[2] = static_cast< GLfloat >( color.blueF() );
        rgba[3] = static_cast< GLfloat >( color.alphaF() );
    }

    color_t( const color_t& color ) : rgba( color.rgba ) {}

    GLfloat red() const   { return rgba[0]; }
    GLfloat green() const { return rgba[1]; }
    GLfloat blue() const  { return rgba[2]; }
    GLfloat alpha() const { return rgba[3]; }
};

inline color_t operator+( const color_t& lhs, const color_t& rhs ) { return color_t( std::min( lhs.red() + rhs.red(), 1.0f ), std::min( lhs.green() + rhs.green(), 1.0f ), std::min( lhs.blue()  + rhs.blue(), 1.0f ), std::min( lhs.alpha() + rhs.alpha(), 1.0f )); }

inline color_t operator*( const color_t& color, double scalar ) { return color_t( color.red() * scalar, color.green() * scalar, color.blue() * scalar, color.alpha() * scalar ); }

struct vertex_t
{
    Eigen::Vector3f position;
    color_t color;
    GLfloat point_size;

    vertex_t(): point_size( 1 ) {}
    vertex_t( const Eigen::Vector3f& position, const color_t& color,GLfloat point_size=1): position( position ), color( color ), point_size( point_size ) {}
    vertex_t( const Eigen::Vector3d& p, const color_t& color,GLfloat point_size=1): position( p.cast< float >() ), color( color ), point_size( point_size ) {}
//     vertex_t( const Eigen::Vector3f& position, const QColor& color )
//         : position( position ), color( color ),point_size(5) {}
};

} } } // namespace snark { namespace graphics { namespace qopengl {

namespace comma { namespace visiting {

template <> struct traits< snark::graphics::qopengl::color_t >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::qopengl::color_t& p, Visitor& v )
    {
        int red   = 0;
        int green = 0;
        int blue  = 0;
        int alpha = 255;
        v.apply( "r", red );
        v.apply( "g", green );
        v.apply( "b", blue );
        v.apply( "a", alpha );
        p = snark::graphics::qopengl::color_t( red, green, blue, alpha );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::qopengl::color_t& p, Visitor& v )
    {
        v.apply( "r", p.red() );
        v.apply( "g", p.green() );
        v.apply( "b", p.blue() );
        v.apply( "a", p.alpha() );
    }
};

// template <> struct traits< snark::graphics::qt3d::vertex_t >
// {
//     template < typename Key, class Visitor >
//     static void visit( Key, snark::graphics::qt3d::vertex_t& p, Visitor& v )
//     {
//         v.apply( "position", p.position );
//         v.apply( "color", p.color );
//     }
// 
//     template < typename Key, class Visitor >
//     static void visit( Key, const snark::graphics::qt3d::vertex_t& p, Visitor& v )
//     {
//         v.apply( "position", p.position );
//         v.apply( "color", p.color );
//     }
// };

} } // namespace comma { namespace visiting {
