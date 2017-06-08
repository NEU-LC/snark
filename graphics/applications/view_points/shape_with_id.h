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

/// @author Vsevolod Vlaskine, Cedric Wohlleber

#pragma once

#include <boost/array.hpp>
#include <boost/optional.hpp>
#include <boost/static_assert.hpp>
#include <comma/base/types.h>
#include <comma/visiting/traits.h>
#include "../../../math/interval.h"
#include "../../../math/rotation_matrix.h"
#include "../../../graphics/block_buffer.h"
#if Qt3D_VERSION==1
#include <Qt3D/qglnamespace.h>
#include <Qt3D/qglpainter.h>
#else
#include <Eigen/Core>
#include "../../../graphics/qt3d/qt3d_v2/gl/shapes.h"
#endif
#include <memory>
#include "types.h"
#include <snark/math/roll_pitch_yaw.h>
#include <snark/visiting/traits.h>
#include "traits.h"

namespace snark { namespace graphics { namespace view {

#if Qt3D_VERSION==1
struct vertex_t
{
    QVector3D position;
    QColor4ub color;
    vertex_t() {}
    vertex_t( const QVector3D& position, const QColor4ub& color )
        : position( position ), color( color ) {}
    vertex_t(const Eigen::Vector3f& p,const QColor4ub& color) : position(p.x(),p.y(),p.z()), color(color) { }
    vertex_t(const Eigen::Vector3d& p,const QColor4ub& color) : position(p.x(),p.y(),p.z()), color(color) { }
};
#else
typedef qt3d::gl::vertex_t vertex_t;
typedef std::shared_ptr<snark::graphics::qt3d::gl::shape> gl_shape_ptr_t;
struct gl_parameters
{
    gl_parameters(unsigned point_size,bool fill) : point_size(point_size),fill(fill) { }
    unsigned point_size;
    bool fill;
};
//static gl_shape_ptr_t make_shape(const gl_parameters& gl)
#endif

namespace detail {

template < typename S > struct shape_traits { static S zero() { return S(); } };
template <> struct shape_traits< Eigen::Vector3d > { static Eigen::Vector3d zero() { return Eigen::Vector3d::Zero(); } };
template <> struct shape_traits< std::pair< Eigen::Vector3d, Eigen::Vector3d > > { static std::pair< Eigen::Vector3d, Eigen::Vector3d > zero() { return std::make_pair( Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero() ); } };

} // namespace detail {

template < class S >
struct ShapeWithId // quick and dirty
{
    typedef S Shape;
    ShapeWithId() : shape( detail::shape_traits< S >::zero() ), id( 0 ), block( 0 ) {}
    ShapeWithId( const S& shape ) : shape( shape ), id( 0 ), block( 0 ), scalar( 0 ), fill( false ) {}
    S shape;
    comma::uint32 id;
    comma::uint32 block;
    color_t color;
    std::string label;
    double scalar;
    bool fill; // todo: just a placeholder for now, plug in if possible or tear down
};

struct how_t { struct points; struct connected; struct loop; }; // quick and dirty; for points only

template < class S, typename How = how_t::points >
struct Shapetraits {}; // quick and dirty

template<>
struct Shapetraits< snark::math::closed_interval< double, 3 > >
{
    static const unsigned int size = 8;
        
#if Qt3D_VERSION==2
    static gl_shape_ptr_t make_shape(const gl_parameters& gl)
    {
        return gl_shape_ptr_t(new snark::graphics::qt3d::gl::shapes::line_strip());
    }
#endif

    #if Qt3D_VERSION==1
    static void update( const snark::math::closed_interval< double, 3 >& e, const Eigen::Vector3d& offset, const QColor4ub& color, unsigned int block, block_buffer< vertex_t >& buffer, boost::optional< snark::math::closed_interval< float, 3 > >& extents  )
    #else
    static void update( const snark::math::closed_interval< double, 3 >& e, const Eigen::Vector3d& offset, const color_t& color, unsigned int block, 
                        block_buffer< vertex_t >& buffer, boost::optional< snark::math::closed_interval< float, 3 > >& extents)
    #endif
    {
        Eigen::Vector3f min = ( e.min() - offset ).cast< float >();
        Eigen::Vector3f max = ( e.max() - offset ).cast< float >();

        #if Qt3D_VERSION==1
        buffer.add( vertex_t( QVector3D( min.x(), min.y(), min.z() ), color ), block );
        buffer.add( vertex_t( QVector3D( min.x(), min.y(), max.z() ), color ), block );
        buffer.add( vertex_t( QVector3D( min.x(), max.y(), max.z() ), color ), block );
        buffer.add( vertex_t( QVector3D( min.x(), max.y(), min.z() ), color ), block );
        buffer.add( vertex_t( QVector3D( max.x(), min.y(), min.z() ), color ), block );
        buffer.add( vertex_t( QVector3D( max.x(), min.y(), max.z() ), color ), block );
        buffer.add( vertex_t( QVector3D( max.x(), max.y(), max.z() ), color ), block );
        buffer.add( vertex_t( QVector3D( max.x(), max.y(), min.z() ), color ), block );
        #else
        buffer.add( vertex_t( Eigen::Vector3f( min.x(), min.y(), min.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( min.x(), min.y(), max.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( min.x(), max.y(), max.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( min.x(), max.y(), min.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( min.x(), min.y(), min.z() ), color ), block );
        
        buffer.add( vertex_t( Eigen::Vector3f( max.x(), min.y(), min.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( max.x(), min.y(), max.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( max.x(), max.y(), max.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( max.x(), max.y(), min.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( max.x(), min.y(), min.z() ), color ), block );
        
        buffer.add( vertex_t( Eigen::Vector3f( max.x(), min.y(), max.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( min.x(), min.y(), max.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( min.x(), max.y(), max.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( max.x(), max.y(), max.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( max.x(), max.y(), min.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( min.x(), max.y(), min.z() ), color ), block );
        #endif

        extents = extents
                ? extents->hull( snark::math::closed_interval< float, 3 >( min, max ) )
                : snark::math::closed_interval< float, 3 >( min, max );
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        const boost::array< unsigned short, 8  > baseIndices = { { 0, 4, 1, 5, 2, 6, 3, 7 } };
        for( unsigned int i = 0; i < size; i += 8 )
        {
            // 2 line loops
            painter->draw( QGL::LineLoop, 4, i );
            painter->draw( QGL::LineLoop, 4, i + 4 );

            // 4 lines
            boost::array< unsigned short, 8  > lineIndices = baseIndices;
            for( unsigned int k = 0; k < lineIndices.size(); ++k ) { lineIndices[k] += i; }
            painter->draw( QGL::Lines, &lineIndices[0], 8 );
        }
    }
    #endif

    static const Eigen::Vector3d& somePoint( const snark::math::closed_interval< double, 3 >& extents ) { return extents.min(); }

    static Eigen::Vector3d center( const snark::math::closed_interval< double, 3 >& extents ) { return ( extents.min() + extents.max() ) / 2; }
};

template <>
struct Shapetraits< std::pair< Eigen::Vector3d, Eigen::Vector3d > >
{
    static const unsigned int size = 2;
    
#if Qt3D_VERSION==2
    static gl_shape_ptr_t make_shape(const gl_parameters& gl)
    {
        return gl_shape_ptr_t(new snark::graphics::qt3d::gl::shapes::lines());
    }
#endif

    #if Qt3D_VERSION==1
    static void update( const std::pair< Eigen::Vector3d, Eigen::Vector3d >& p, const Eigen::Vector3d& offset, const QColor4ub& color, unsigned int block, block_buffer< vertex_t >& buffer, boost::optional< snark::math::closed_interval< float, 3 > >& extents  )
    #else
    static void update( const std::pair< Eigen::Vector3d, Eigen::Vector3d >& p, const Eigen::Vector3d& offset, const color_t& color, unsigned int block, 
                        block_buffer< vertex_t >& buffer, boost::optional< snark::math::closed_interval< float, 3 > >& extents)
    #endif
    {
        Eigen::Vector3f first = ( p.first - offset ).cast< float >();
        Eigen::Vector3f second = ( p.second - offset ).cast< float >();
        //if(    comma::math::equal( first.x(), second.x() )
        //    && comma::math::equal( first.y(), second.y() )
        //    && comma::math::equal( first.z(), second.z() ) ) { return; } // todo: draw a point instead?
        #if Qt3D_VERSION==1
        buffer.add( vertex_t( QVector3D( first.x(), first.y(), first.z() ), color ), block );
        buffer.add( vertex_t( QVector3D( second.x(), second.y(), second.z() ), color ), block );
        #else
        buffer.add( vertex_t( Eigen::Vector3f( first.x(), first.y(), first.z() ), color ), block );
        buffer.add( vertex_t( Eigen::Vector3f( second.x(), second.y(), second.z() ), color ), block );
        #endif

        extents = extents
                ? extents->hull( snark::math::closed_interval< float, 3 >( first ) )
                : snark::math::closed_interval< float, 3 >( first );
        extents = extents->hull( snark::math::closed_interval< float, 3 >( second ) );
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill ) { painter->draw( QGL::Lines, size ); }
    #endif

    static const Eigen::Vector3d& somePoint( const std::pair< Eigen::Vector3d, Eigen::Vector3d >& line ) { return line.first; }

    static Eigen::Vector3d center( const std::pair< Eigen::Vector3d, Eigen::Vector3d >& line ) { return ( line.first + line.second ) / 2; }
};

template < std::size_t Size >
struct loop { boost::array< Eigen::Vector3d, Size > corners; };

template < std::size_t Size >
struct Shapetraits< loop< Size > >
{
    BOOST_STATIC_ASSERT( Size > 2 );
    static const unsigned int size = Size;
    
#if Qt3D_VERSION==2
    static gl_shape_ptr_t make_shape(const gl_parameters& gl)
    {
        return gl_shape_ptr_t(new snark::graphics::qt3d::gl::shapes::triangles(gl.fill));
    }
#endif

    #if Qt3D_VERSION==1
    static void update( const loop< Size >& e, const Eigen::Vector3d& offset, const QColor4ub& color, unsigned int block, block_buffer< vertex_t >& buffer, boost::optional< snark::math::closed_interval< float, 3 > >& extents  )
    #else
    static void update( const loop< Size >& e, const Eigen::Vector3d& offset, const color_t& color, unsigned int block, 
                        block_buffer< vertex_t >& buffer, boost::optional< snark::math::closed_interval< float, 3 > >& extents)
    #endif
    {
        for( unsigned int i = 0; i < Size; ++i )
        {
            Eigen::Vector3f v = ( e.corners[i] - offset ).template cast< float >();
            
            #if Qt3D_VERSION==1
            buffer.add( vertex_t( QVector3D( v.x(), v.y(), v.z() ), color ), block );
            #else
            buffer.add( vertex_t( v, color ), block );
            #endif

            extents = extents ? extents->hull( snark::math::closed_interval< float, 3 >( v ) ) : snark::math::closed_interval< float, 3 >( v );
        }
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        for( unsigned int i = 0; i < size; i += Size ) { painter->draw( fill && Size == 3 ? QGL::Triangles : QGL::LineLoop, Size, i ); }
    }
    #endif
    
    static const Eigen::Vector3d& somePoint( const loop< Size >& c ) { return c.corners[0]; }
    
    static Eigen::Vector3d center( const loop< Size >& c ) // quick and dirty
    { 
        Eigen::Vector3d m = Eigen::Vector3d::Zero();
        for( unsigned int i = 0; i < Size; ++i ) { m += c.corners[i]; }
        return m / Size;
    }
};

template < std::size_t Size >
struct Ellipse
{
    Eigen::Vector3d center;
    Eigen::Vector3d orientation;
    Ellipse() : center( 0, 0, 0 ), orientation( 0, 0, 0 ) {}
    double major;
    double minor;
};

template < std::size_t Size >
struct Shapetraits< Ellipse< Size > >
{
    static const unsigned int size = Size;
    
#if Qt3D_VERSION==2
    static gl_shape_ptr_t make_shape(const gl_parameters& gl)
    {
        return gl_shape_ptr_t(new snark::graphics::qt3d::gl::shapes::line_loop());
    }
#endif

    #if Qt3D_VERSION==1
    static void update( const Ellipse< Size >& ellipse, const Eigen::Vector3d& offset, const QColor4ub& color, unsigned int block, block_buffer< vertex_t >& buffer, boost::optional< snark::math::closed_interval< float, 3 > >& extents  )
    #else
    static void update( const Ellipse< Size >& ellipse, const Eigen::Vector3d& offset, const color_t& color, unsigned int block, 
                        block_buffer< vertex_t >& buffer, boost::optional< snark::math::closed_interval< float, 3 > >& extents)
    #endif
    {
        Eigen::Vector3d c = ellipse.center - offset;
        const Eigen::Matrix3d& r = rotation_matrix::rotation( ellipse.orientation );
        static const double step = 3.14159265358979323846l * 2 / Size;
        double angle = 0;
        // todo: use native opengl rotation and normals instead
        for( std::size_t i = 0; i < Size; ++i, angle += step )
        {
            Eigen::Vector3d v = r * Eigen::Vector3d( std::cos( angle ) * ellipse.major, std::sin( angle ) * ellipse.minor, 0 );
            Eigen::Vector3d p( v.x(), v.y(), v.z() );
            Eigen::Vector3f point = ( p + c ).cast< float >();
            #if Qt3D_VERSION==1
            buffer.add( vertex_t( QVector3D( point.x(), point.y(), point.z() ), color ), block );
            #else
            buffer.add( vertex_t( point, color ), block );
            #endif
            extents = extents ? extents->hull( point ) : snark::math::closed_interval< float, 3 >( point );
        }
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        for( unsigned int i = 0; i < size; i += Size )
        {
            painter->draw( QGL::LineLoop, Size, i );
        }
    }
    #endif

    static const Eigen::Vector3d& somePoint( const Ellipse< Size >& ellipse ) { return ellipse.center; }

    static Eigen::Vector3d center( const Ellipse< Size >& ellipse ) { return ellipse.center; }
};

template < std::size_t Size >
struct arc // todo: quick and dirty; generalize for ellipse; and maybe get rid of ellipse class
{
    Eigen::Vector3d begin;
    boost::optional< Eigen::Vector3d > middle;
    Eigen::Vector3d end;
    Eigen::Vector3d centre;
    arc() : centre( 0, 0, 0 ) {}
};

template < std::size_t Size >
struct Shapetraits< arc< Size > >
{
    static const unsigned int size = Size;

    BOOST_STATIC_ASSERT( Size % 2 == 0 ); // quick and dirty: for simplicity support only only even sizes
    
#if Qt3D_VERSION==2
    static gl_shape_ptr_t make_shape(const gl_parameters& gl)
    {
        return gl_shape_ptr_t(new snark::graphics::qt3d::gl::shapes::line_strip());
    }
#endif

    #if Qt3D_VERSION==1
    static void update( const arc< Size >& a, const Eigen::Vector3d& offset, const QColor4ub& color, unsigned int block, block_buffer< vertex_t >& buffer, boost::optional< snark::math::closed_interval< float, 3 > >& extents  )
    #else
    static void update( const arc< Size >& a, const Eigen::Vector3d& offset, const color_t& color, unsigned int block, 
                        block_buffer< vertex_t >& buffer, boost::optional< snark::math::closed_interval< float, 3 > >& extents)
    #endif
    {
        if( ( a.begin - a.end ).squaredNorm() < ( 0.001 * 0.001 ) ) // real quick and dirty: if begin and end coincide
        {
            Eigen::Vector3d v = a.begin;
            Eigen::Vector3d step = ( a.end - a.begin ) / Size;
            for( std::size_t i = 0; i < Size; ++i, v += step ) // just to make the gl buffer sizes right
            {
                Eigen::Vector3f point = ( v - offset ).cast< float >();
                #if Qt3D_VERSION==1
                buffer.add( vertex_t( QVector3D( point.x(), point.y(), point.z() ), color ), block );
                #else
                buffer.add( vertex_t( point, color ), block );
                #endif
                extents = extents ? extents->hull( point ) : snark::math::closed_interval< float, 3 >( point );
            }
            return;
        }
        // get centre
        Eigen::Vector3d centre;
        Eigen::Vector3d normal;
        if( a.middle )
        {
            Eigen::Vector3d begin_middle = a.begin - *a.middle;
            Eigen::Vector3d middle_end = *a.middle - a.end;
            Eigen::Vector3d end_begin = a.end - a.begin;
            normal = begin_middle.cross( middle_end );
            double k = normal.squaredNorm() * 2;
            double alpha = -middle_end.squaredNorm() * begin_middle.dot( end_begin ) / k;
            double beta = -end_begin.squaredNorm() * middle_end.dot( begin_middle ) / k;
            double gamma = -begin_middle.squaredNorm() * end_begin.dot( middle_end ) / k;
            centre = a.begin * alpha + *a.middle * beta + a.end * gamma;
        }
        else
        {
            centre = a.centre;
            normal = ( a.begin - a.centre ).cross( a.end - a.centre );
        }
        normal.normalize();
        // get rotation from begin to end with Size steps
        Eigen::Vector3d b = a.begin - centre;
        Eigen::Vector3d e = a.end - centre;
        Eigen::AngleAxis< double > aa( Eigen::Quaternion< double >::FromTwoVectors( b, e ) );
        Eigen::AngleAxis< double > nn( aa.angle() / ( Size - 1 ), normal );
        const Eigen::Matrix3d& r = nn.toRotationMatrix();
        Eigen::Vector3d c = centre - offset;
        Eigen::Vector3d v = b;
        for( std::size_t i = 0; i < Size; ++i, v = r * v ) // todo: use native opengl rotation and normals instead
        {
            Eigen::Vector3f point = ( v + c ).cast< float >();
            #if Qt3D_VERSION==1
            buffer.add( vertex_t( QVector3D( point.x(), point.y(), point.z() ), color ), block );
            #else
            buffer.add( vertex_t( point, color ), block );
            #endif
            extents = extents ? extents->hull( point ) : snark::math::closed_interval< float, 3 >( point );
        }
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        for( unsigned int i = 0; i < size; i += Size )
        {
            painter->draw( QGL::Lines, Size, i );
            painter->draw( QGL::Lines, Size - 1, i + 1 );
        }
    }
    #endif

    static const Eigen::Vector3d& somePoint( const arc< Size >& a ) { return a.begin; }

    static Eigen::Vector3d center( const arc< Size >& a ) { return a.middle ? *a.middle : a.centre; } // quick and dirty
    //static Eigen::Vector3d center( const arc< Size >& a ) { return a.middle; } // quick and dirty
};

#if Qt3D_VERSION==1
template < typename How > struct draw_traits_;

template <> struct draw_traits_< how_t::points >
{
    static const QGL::DrawingMode drawing_mode = QGL::Points;

    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        painter->draw( QGL::Points, size );
    }
};

template <> struct draw_traits_< how_t::loop >
{
    static const QGL::DrawingMode drawing_mode = QGL::DrawingMode();

    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        painter->draw( QGL::LineLoop, size );
    }
};

template <> struct draw_traits_< how_t::connected >
{
    static const QGL::DrawingMode drawing_mode = QGL::DrawingMode();

    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        painter->draw( QGL::Lines, size );
        if( size > 1 ) { painter->draw( QGL::Lines, size - 1, 1 ); }
    }
};
#elif Qt3D_VERSION==2

template<typename T> struct how_traits
{
//     static gl_shape_ptr_t make_shape(const gl_parameters& gl) { return gl_shape_ptr_t(); }
};
template<> struct how_traits<how_t::points>
{
    static gl_shape_ptr_t make_shape(const gl_parameters& gl)
    {
        return gl_shape_ptr_t(new snark::graphics::qt3d::gl::shapes::point(gl.point_size));
    }
};
template<> struct how_traits<how_t::loop>
{
    static gl_shape_ptr_t make_shape(const gl_parameters& gl)
    {
        return gl_shape_ptr_t(new snark::graphics::qt3d::gl::shapes::line_loop());
    }
};
template<> struct how_traits<how_t::connected>
{
    static gl_shape_ptr_t make_shape(const gl_parameters& gl)
    {
        return gl_shape_ptr_t(new snark::graphics::qt3d::gl::shapes::line_strip());
    }
};

#endif

template< typename How >
struct Shapetraits< Eigen::Vector3d, How >
{
    #if Qt3D_VERSION==1
    static const QGL::DrawingMode drawingMode = draw_traits_< How >::drawing_mode;
    #endif
    static const unsigned int size = 1;

    
#if Qt3D_VERSION==2
    static gl_shape_ptr_t make_shape(const gl_parameters& gl)
    {
        return how_traits<How>::make_shape(gl);
    }
#endif

    #if Qt3D_VERSION==1
    static void update( const Eigen::Vector3d& p, const Eigen::Vector3d& offset, const QColor4ub& color, unsigned int block, block_buffer< vertex_t >& buffer, boost::optional< snark::math::closed_interval< float, 3 > >& extents  )
    #else
    static void update( const Eigen::Vector3d& p, const Eigen::Vector3d& offset, const color_t& color, unsigned int block, 
                        block_buffer< vertex_t >& buffer, boost::optional< snark::math::closed_interval< float, 3 > >& extents)
    #endif
    {
        Eigen::Vector3d point = p - offset;
        #if Qt3D_VERSION==1
        buffer.add( vertex_t( QVector3D( point.x(), point.y(), point.z() ), color ), block );
        #else
        buffer.add( vertex_t( Eigen::Vector3f( point.x(), point.y(), point.z() ), color), block );
        #endif
        extents = extents
                ? extents->hull( point.cast< float >() )
                : snark::math::closed_interval< float, 3 >( point.cast< float >() );
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill ) { draw_traits_< How >::draw( painter, size, fill ); }
    #endif

    static const Eigen::Vector3d& somePoint( const Eigen::Vector3d& point ) { return point; }

    static const Eigen::Vector3d& center( const Eigen::Vector3d& point ) { return point; }
};


struct axis
{
    Eigen::Vector3d position;
    snark::roll_pitch_yaw orientation;
    double length;
    axis() : position( 0, 0, 0 ), orientation( 0, 0, 0 ), length(1) {}
};


template<>
struct Shapetraits< axis >
{
    static const unsigned int size = 6;
#if Qt3D_VERSION==2
    static gl_shape_ptr_t make_shape(const gl_parameters& gl)
    {
        return gl_shape_ptr_t(new snark::graphics::qt3d::gl::shapes::lines(gl.point_size));
    }
#endif

    static void update( const axis& axis, const Eigen::Vector3d& offset, const color_t& color, unsigned int block, 
                        block_buffer< vertex_t >& buffer, boost::optional< snark::math::closed_interval< float, 3 > >& extents)
    {
        Eigen::Vector3d pos=axis.position - offset;
        Eigen::Matrix3d ori=rotation_matrix::rotation( axis.orientation );
        Eigen::Vector3d una(axis.length,0,0);
        Eigen::Vector3d p=pos + ori*una;
        buffer.add( vertex_t(pos,COLOR_RED),block);
        buffer.add( vertex_t(p,COLOR_RED),block);
//         ori=rotation_matrix::rotation( Eigen::Vector3d(axis.orientation.x(), axis.orientation.y()+M_PI/2, axis.orientation.z()));
        Eigen::Matrix3d y_r=rotation_matrix::rotation(Eigen::Vector3d(0,0,M_PI/2));
        p=pos+ori*y_r*una;
        buffer.add( vertex_t(pos,COLOR_GREEN),block);
        buffer.add( vertex_t(p,COLOR_GREEN),block);
//         ori=rotation_matrix::rotation( Eigen::Vector3d(axis.orientation.x(), axis.orientation.y(), axis.orientation.z()+M_PI/2));
        Eigen::Matrix3d z_r=rotation_matrix::rotation(Eigen::Vector3d(0,-M_PI/2,0));
        p=pos+ori*z_r*una;
        buffer.add( vertex_t(pos,COLOR_BLUE),block);
        buffer.add( vertex_t(p,COLOR_BLUE),block);

        extents = extents ? extents->hull( pos.cast<float>() ) : snark::math::closed_interval< float, 3 >( pos.cast<float>() );
        extents->hull( p.cast<float>() );
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        painter->draw(QGL::Lines, size);
    }
    #endif

    static const Eigen::Vector3d& somePoint( const axis& axis ) { return axis.position; }

    static Eigen::Vector3d center( const axis& axis ) { return axis.position; }
};


} } } // namespace snark { namespace graphics { namespace view {

namespace comma { namespace visiting {

template < typename S > struct traits< snark::graphics::view::ShapeWithId< S > >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::view::ShapeWithId< S >& p, Visitor& v )
    {
        v.apply( "shape", p.shape );
        v.apply( "id", p.id );
        v.apply( "block", p.block );
        v.apply( "colour", p.color );
        v.apply( "label", p.label );
        v.apply( "scalar", p.scalar );
        v.apply( "fill", p.fill );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::view::ShapeWithId< S >& p, Visitor& v )
    {
        v.apply( "shape", p.shape );
        v.apply( "id", p.id );
        v.apply( "block", p.block );
        v.apply( "colour", p.color );
        v.apply( "label", p.label );
        v.apply( "scalar", p.scalar );
        v.apply( "fill", p.fill );
    }
};

template < std::size_t Size > struct traits< snark::graphics::view::Ellipse< Size > >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::view::Ellipse< Size >& p, Visitor& v )
    {
        v.apply( "center", p.center );
        v.apply( "orientation", p.orientation );
        v.apply( "major", p.major );
        v.apply( "minor", p.minor );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::view::Ellipse< Size >& p, Visitor& v )
    {
        v.apply( "center", p.center );
        v.apply( "orientation", p.orientation );
        v.apply( "major", p.major );
        v.apply( "minor", p.minor );
    }
};

template < std::size_t Size > struct traits< snark::graphics::view::arc< Size > >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::view::arc< Size >& p, Visitor& v )
    {
        v.apply( "begin", p.begin );
        if( p.middle ) { v.apply( "middle", *p.middle ); } // quick and dirty
        v.apply( "end", p.end );
        v.apply( "centre", p.centre );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::view::arc< Size >& p, Visitor& v )
    {
        v.apply( "begin", p.begin );
        if( p.middle ) { v.apply( "middle", *p.middle ); } // quick and dirty
        v.apply( "end", p.end );
        v.apply( "centre", p.centre );
    }
};

template < typename T > struct traits< snark::math::closed_interval< T, 3 > >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::math::closed_interval< T, 3 >& p, Visitor& v )
    {
        Eigen::Matrix< T, 3, 1 > min;
        Eigen::Matrix< T, 3, 1 > max;
        v.apply( "min", min );
        v.apply( "max", max );
        p = snark::math::closed_interval< T, 3 >( min, max );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::math::closed_interval< T, 3 >& p, Visitor& v )
    {
        v.apply( "min", p.min() );
        v.apply( "max", p.max() );
    }
};

template < std::size_t Size > struct traits< snark::graphics::view::loop< Size > >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::view::loop< Size >& p, Visitor& v )
    {
        v.apply( "corners", p.corners );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::view::loop< Size >& p, Visitor& v )
    {
        v.apply( "corners", p.corners );
    }
};

template<>
struct traits< snark::graphics::view::axis >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::view::axis& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
        v.apply( "length", p.length );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::view::axis& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
        v.apply( "length", p.length );
    }
};


} } // namespace comma { namespace visiting {
