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

#pragma once

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include "../../camera_options.h"
#include "shapes.h"
#include "label_shader.h"
#include <vector>
#include <memory>
#include "camera.h"

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)

namespace snark { namespace graphics { namespace qt3d { namespace gl {

class widget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    widget(const camera_options& camera_options, QWidget *parent = 0 );
    ~widget();

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;

    std::vector<std::shared_ptr<shape>> shapes;
    std::vector<std::shared_ptr<label_shader>> label_shaders;
    
    void begin_update();
    void end_update();
public slots:
    void cleanup();
    
protected:
    virtual void init() {};

    void initializeGL() Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;
    void resizeGL( int width, int height ) Q_DECL_OVERRIDE;
    void mousePressEvent( QMouseEvent *event ) Q_DECL_OVERRIDE;
    void mouseMoveEvent( QMouseEvent *event ) Q_DECL_OVERRIDE;
    void mouseDoubleClickEvent( QMouseEvent *event ) Q_DECL_OVERRIDE;
    void wheelEvent( QWheelEvent *event ) Q_DECL_OVERRIDE;
    
//     void set_near_plane(float near_plane);
    void set_far_plane(float f);

protected:
    void update_projection();

    boost::optional< QVector3D > viewport_to_3d( const QPoint& point_2d );
    boost::optional< QVector3D > pixel_at_point( const QPoint& viewport_point, int search_width );
    boost::optional< QVector3D > pixel_nearest_centre( const std::vector< float >& depth, int search_width );

    QPoint last_pos_;
    QOpenGLShaderProgram *program_;
    int projection_matrix_location_;
    int mv_matrix_location_;
    camera_transform camera;
    camera_options camera_options_;
    double size_;
public:
    float near_plane;
    float far_plane;
    double scene_radius;
};

} } } } // namespace snark { namespace graphics { namespace qt3d { namespace gl {

