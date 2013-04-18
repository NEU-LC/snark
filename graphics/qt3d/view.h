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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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

#endif /*SNARK_GRAPHICS_GL_VIEW_H_*/
