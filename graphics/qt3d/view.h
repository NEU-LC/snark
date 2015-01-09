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


/// @author Cedric Wohlleber

#ifndef SNARK_GRAPHICS_GL_VIEW_H_
#define SNARK_GRAPHICS_GL_VIEW_H_

#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <Eigen/Core>
#include <Qt3D/qglview.h>
#include <QMouseEvent>
#include <comma/visiting/traits.h> // quick and dirty
#include "./coordinates.h"

namespace snark { namespace graphics { namespace qt3d {

/// base class for 3d viewers with mouse navigation
class view : public QGLView
{
    Q_OBJECT
public:
    view( const QColor4ub& background_color
        , double fov
        , bool z_up
        , bool orthographic = false
        , boost::optional< QVector3D > scene_center = boost::optional< QVector3D >()
        , boost::optional< double > scene_radius = boost::optional< double >() );

    virtual ~view() {}

    double scene_radius() const;

private slots:
    void hide_coordinates() { m_show_coordinates = false; update(); }

protected:
    void updateZFar();
    void updateView( const QVector3D& min, const QVector3D& max );
    void lookAtCenter();
    void draw_coordinates( QGLPainter* painter );
    void mousePressEvent( QMouseEvent *e );
    void mouseReleaseEvent( QMouseEvent *e );
    void mouseMoveEvent( QMouseEvent *e );
    void wheelEvent( QWheelEvent *e );
    QVector3D unproject( float x, float y, float depth );
    boost::optional< QVector3D > getPoint( const QPoint& point2d );
    void mouseDoubleClickEvent( QMouseEvent *e );

    const QColor4ub m_background_color;
    QVector3D m_sceneCenter;
    bool scene_center_fixed_;
    bool m_z_up;
    boost::optional< Eigen::Vector3d > m_offset;

private:
    boost::optional< QPoint > m_startPan;
    boost::optional< QPoint > m_startRotate;
    double scene_radius_;
    bool scene_radius_fixed_;
    QVector3D m_revolve;
    boost::optional< coordinates > m_coordinates;
    bool m_show_coordinates;
};

} } } // namespace snark { namespace graphics { namespace gt3d {

namespace comma { namespace visiting {

template <> struct traits< QVector3D >
{
    template < typename Key, class Visitor >
    static void visit( Key, QVector3D& p, Visitor& v )
    {
        double d;
        d= p.x(); v.apply( "x", d ); p.setX( d );
        d = p.y(); v.apply( "y", d ); p.setY( d );
        d = p.z(); v.apply( "z", d ); p.setZ( d );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const QVector3D& p, Visitor& v )
    {
        v.apply( "x", p.x() );
        v.apply( "y", p.y() );
        v.apply( "z", p.z() );
    }
};

template <> struct traits< QSizeF >
{
    template < typename Key, class Visitor >
    static void visit( Key, QSizeF& p, Visitor& v )
    {
        double d;
        d = p.width(); v.apply( "width", d ); p.setWidth( d );
        d = p.height(); v.apply( "height", d ); p.setHeight( d );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const QSizeF& p, Visitor& v )
    {
        v.apply( "width", p.width() );
        v.apply( "height", p.height() );
    }
};

template <> struct traits< QGLCamera >
{
    template < typename Key, class Visitor >
    static void visit( Key, QGLCamera& p, Visitor& v )
    {
        bool b = p.adjustForAspectRatio(); v.apply( "adjust_for_aspect_ratio", b ); p.setAdjustForAspectRatio( b );
        QVector3D w;
        w = p.center(); v.apply( "center", w ); p.setCenter( w );
        w = p.eye(); v.apply( "eye", w ); p.setEye( w );
        double d;
        d = p.eyeSeparation(); v.apply( "eye-separation", d ); p.setEyeSeparation( d );
        d = p.farPlane(); v.apply( "far-plane", d ); p.setFarPlane( d );
        d = p.fieldOfView(); v.apply( "field-of-view", d ); p.setFieldOfView( d );
        QSizeF s;
        s = p.minViewSize(); v.apply( "min-view-size", s ); p.setMinViewSize( s );
        w = p.motionAdjustment(); v.apply( "motion-adjustment", w ); p.setMotionAdjustment( w );
        d = p.nearPlane(); v.apply( "near-plane", d ); p.setNearPlane( d );
        int i;
        i = p.projectionType(); v.apply( "projection-type", i ); p.setProjectionType( static_cast< QGLCamera::ProjectionType >( i ) );
        i = p.screenRotation(); v.apply( "screen-rotation", i ); p.setScreenRotation( i );
        w = p.upVector(); v.apply( "up-vector", w ); p.setUpVector( w );
        s = p.viewSize(); v.apply( "view-size", s ); p.setViewSize( s );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const QGLCamera& p, Visitor& v )
    {
        v.apply( "adjust_for_aspect_ratio", p.adjustForAspectRatio() );
        v.apply( "center", p.center() );
        v.apply( "eye", p.eye() );
        v.apply( "eye-separation", p.eyeSeparation() );
        v.apply( "far-plane", p.farPlane() );
        v.apply( "field-of-view", p.fieldOfView() );
        v.apply( "min-view-size", p.minViewSize() );
        v.apply( "motion-adjustment", p.motionAdjustment() );
        v.apply( "near-plane", p.nearPlane() );
        v.apply( "projection-type", static_cast< int >( p.projectionType() ) );
        v.apply( "screen-rotation", p.screenRotation() );
        v.apply( "up-vector", p.upVector() );
        v.apply( "view-size", p.viewSize() );
    }
};

} } // namespace comma { namespace visiting {

#endif /*SNARK_GRAPHICS_GL_VIEW_H_*/
